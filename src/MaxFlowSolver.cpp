#include <algorithm>
#include <iostream>
#include <vector>
#include "MaxFlowSolver.h"

using namespace std;

double MaxFlowSolver::augmentFlow(const vector<int>& path,
        vector<vector<double>>& residual) {
    if (path[0]!=s || path.back()!=t) {
        cout<<"invalid augmenting path"<<endl;
        cin.get();
    }
    double mincap=1e+8;
    for (int i=1; i<path.size(); ++i)
        if (residual[path[i-1]][path[i]]<mincap)
            mincap=residual[path[i-1]][path[i]];
    for (int i=1; i<path.size(); ++i) {
        residual[path[i-1]][path[i]]-=mincap;
        residual[path[i]][path[i-1]]+=mincap;
    }
    return mincap;
}

vector<int> MaxFlowSolver::findAugmentingPath(
        const std::vector<std::vector<double>>& residual) {
    // definition
    struct Label {
        int node;
        int idxpred;
        Label(int n, int idx) : node{n}, idxpred{idx} {}
    };
    // initialization
    vector<Label> labels;
    labels.push_back(Label(s, -1));
    vector<int> visited(network.size(), 0);
    visited[s]=1;
    int nextidx=0;
    // loop
    while (nextidx<labels.size()) {
        Label currlabel=labels[nextidx];
        for (int j=0; j<network.size(); ++j) {
            if (j!=currlabel.node && residual[currlabel.node][j]>1e-6
                        && !visited[j]) {
                if (j==t) {     // augmenting path found
                    vector<int> revpath {t};
                    while (nextidx!=-1) {
                        revpath.push_back(labels[nextidx].node);
                        nextidx=labels[nextidx].idxpred;
                    }
                    std::reverse(revpath.begin(), revpath.end());
                    return revpath;
                }
                visited[j]=1;
                labels.push_back(Label(j, nextidx));
            }
        }
        nextidx++;
    }
    return vector<int>();
}

vector<int> MaxFlowSolver::findCut(const vector<vector<double>>& residual) {
    vector<int> cut(network.size(), 0);
    cut[s]=1;
    vector<int> next {s};
    int nextidx=0;
    while (nextidx<next.size()) {
        int curr=next[nextidx];
        for (int j=0; j<network.size(); ++j) {
            if (j!=curr && residual[curr][j]>1e-5 && cut[j]==0) {
                cut[j]=1;
                next.push_back(j);
            }
        }
        nextidx++;
    }
    return cut;
}

MaxFlowSolution MaxFlowSolver::solve() {
    double maxflow=0;
    vector<vector<double>> residual=network;
    while (true) {
        vector<int> augpath=findAugmentingPath(residual);
        if (augpath.size()==0)
            break;
        maxflow+=augmentFlow(augpath, residual);
    }
    return MaxFlowSolution(maxflow, findCut(residual));
}

