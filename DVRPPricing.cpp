#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include "DVRPLabel.h"
#include "DVRPLinearSolver.h"
#include "DVRPPricing.h"

using namespace std;

DVRPPricingState DVRPPricing::state;

void DVRPPricing::addRootLabels(vector<vector<DVRPLabel>>& labels) const {
    for (int i=1; i<data->N; ++i)
        labels[i].emplace_back(vector<int>({0,i}), data->costs[0][i],
                sol.kDual+sol.pttDuals[i]);
}

vector<vector<DVRPLabel>>& DVRPPricing::initLabels() const {
    if (state.firstrun) {
        assert(state.labels.size()==0);
        state.firstrun=false;
        state.labels.insert(state.labels.end(), data->N, vector<DVRPLabel>());
        addRootLabels(state.labels);
    } else {
        assert(state.labels.size()==data->N);
        // recycling: update the cost of all labels saved in the state
        for (int i=1; i<data->N; ++i)
            for (auto& l : state.labels[i])
                l.updateCost(sol.kDual, sol.pttDuals);
    }
    return state.labels;
}

void DVRPPricing::removeDominated(vector<DVRPLabel>& labels) {
    labels.erase(remove_if(labels.begin(), labels.end(),
            [](const DVRPLabel& l){return l.dominated;}), labels.end());
}

void DVRPPricing::setDominated(vector<DVRPLabel>& labels) {
    sort(labels.begin(), labels.end(),
            [](const DVRPLabel& l1, const DVRPLabel& l2)
            {return l1.cost-l1.duals<l2.cost-l2.duals;});
    for (int i=0; i<labels.size(); ++i) {
        const DVRPLabel& l1=labels[i];
        if (!l1.dominated)
            for (int j=i+1; j<labels.size(); ++j) {
                DVRPLabel& l2=labels[j];
                if (!l2.dominated && l1.dominates(l2))
                    l2.dominated=true;
            }
    }
}

vector<DVRPRoute> DVRPPricing::solveExact() const {
    assert(state.labels.size()==0);
    vector<DVRPRoute> routes;
    cout<<"DVRPPricing: solveExact(): initiating exhaustive search"<<endl;
    vector<vector<DVRPLabel>> labels(data->N, vector<DVRPLabel>());
    addRootLabels(labels);
    bool labelsLeft=true;
    while (labelsLeft) {
        labelsLeft=false;
        for (int i=1; i<data->N; ++i) {
            /*
            for (int j=1; j<data->N; ++j)
                removeDominated(labels[j]);
            */
            setDominated(labels[i]);
            for (auto& l : labels[i]) {
                if (l.dominated)
                    continue;
                for (int j=1; j<data->N; ++j) {
                    if (j==i||l.ngForbid(j)||
                        l.cost+data->costs[i][j]+data->costs[j][0]>data->limit)
                        continue;
                    DVRPLabel ext=l.extend(j,data->costs[i][j],sol.pttDuals[j]);
                    double c=ext.cost+data->costs[j][0];
                    if (c-ext.duals<-my0) {
                        auto v=ext.nodes;
                        v.push_back(0);
                        routes.emplace_back(c, move(v), c-ext.duals);
                        if (routes.size()==DVRPLinearSolver::cols_exact)
                            return routes;
                    }
                    labels[j].push_back(move(ext));
                    labelsLeft=true;
                }
            }
            labels[i].clear();
        }
    }
    return routes;
}

vector<DVRPRoute> DVRPPricing::solveHeuristic() const {
    vector<DVRPRoute> routes;
    vector<vector<DVRPLabel>>& labels=initLabels();     // recycling labels
    bool labelsLeft=true;
    while (labelsLeft) {
        labelsLeft=false;
        vector<int> shuffled(data->N-1);
        iota(shuffled.begin(), shuffled.end(), 1);
        random_shuffle(shuffled.begin(), shuffled.end());
        for (int i : shuffled) {
            if (routes.size()>=DVRPLinearSolver::cols_heur) {
                for (int j=1; j<data->N; ++j) {
                    setDominated(labels[j]);
                    removeDominated(labels[j]);
                }
                return routes;
            }
            setDominated(labels[i]);
            for (auto& l : labels[i]) {
                if (l.extended||l.dominated)
                    continue;
                for (int j=1; j<data->N; ++j) {
                    if (j==i||l.ngForbid(j)||
                        l.cost+data->costs[i][j]+data->costs[j][0]>data->limit)
                        continue;
                    DVRPLabel ext=l.extend(j,data->costs[i][j],sol.pttDuals[j]);
                    double c=ext.cost+data->costs[j][0];
                    if (c-ext.duals<-my0) {
                        auto v=ext.nodes;
                        v.push_back(0);
                        routes.emplace_back(c, move(v), c-ext.duals);
                        break;
                    }
                    labels[j].push_back(move(ext));
                    labelsLeft=true;
                }
                l.extended=true;
            }
            labels[i].clear();
        }
    }
    resetState();
    return routes;
}

