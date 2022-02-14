#include <algorithm>
#include <queue>
#include <vector>
#include "Dijkstra.h"

using namespace std;

shared_ptr<Data> Dijkstra::data;
vector<PathTree> Dijkstra::pathtrees;
vector<vector<double>> Dijkstra::costs;

Dijkstra::Dijkstra() {
    assert(data!=nullptr);
    if (pathtrees.size()==0) {
        pathtrees=vector<PathTree>(data->instance().sizeV(), vector<int>());
        costs=vector<vector<double>>(data->instance().sizeV(),
                vector<double>(data->instance().sizeV(), -1));
    }
}

double Dijkstra::cost(int s, int t) {
    if (costs[s][t]==-1)
        solve(s);
    //assert(costs[s][t]!=-1);
    return costs[s][t];
}

Path Dijkstra::solve(int s, int t) {
    assert(s>=0&&s<data->instance().sizeV()&&t>=0&&t<data->instance().sizeV());
    if (s==t)
        return Path(s);
    if (pathtrees[s].preds.size()==0)
        pathtrees[s]=solve(s);
    const auto& pred=pathtrees[s].preds;
    vector<Link> lnks;
    lnks.reserve(256);      // optimized for Vienna network
    const auto& links=data->instance().links();
    lnks.push_back(links[pred[t]]);
    double cost=links[pred[t]].d;
    int prev=links[pred[t]].s;
    while (prev!=s) {
        lnks.push_back(links[pred[prev]]);
        cost+=links[pred[prev]].d;
        prev=links[pred[prev]].s;
    }
    reverse(begin(lnks), end(lnks));
    return {lnks, cost};
}

PathTree Dijkstra::solve(int s) {
    assert(s>=0&&s<data->instance().sizeV());
    vector<int> pred(data->instance().sizeV(), -1);
    vector<double> cost(data->instance().sizeV(),
            numeric_limits<double>::infinity());    // cost=0 => visited
    typedef pair<double, int> validxp;
    typedef priority_queue<validxp, vector<validxp>,
            std::greater<validxp>> mypq;
    mypq pq;
    pq.push({0, s});
    cost[s]=0;
    int done=0;
    while (done<data->instance().sizeV()) {
        auto next=pq.top();
        pq.pop();
        if (next.second!=s && cost[next.second]==0)
            continue;
        costs[s][next.second]=next.first;
        done++;
        for (const auto& l : data->instance().linksFrom(next.second))
            if (next.first+l.d<cost[l.t]) {
                cost[l.t]=next.first+l.d;
                pred[l.t]=l.idx;
                pq.push({cost[l.t], l.t});
            }
        cost[next.second]=0;        // to indicate that it's done
    }
    return pred;
}

