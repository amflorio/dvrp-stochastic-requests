#include <iostream>
#include "Dijkstra.h"
#include "OfflinePlanner.h"

using namespace std;

shared_ptr<Data> OfflinePlanner::data;

vector<vector<double>> OfflinePlanner::genGraph() const {
    vector<vector<double>> tts(reqs.size()+1,vector<double>(reqs.size()+1,0));
    Dijkstra dijk;
    const int dep=data->instance().depot();
    for (int i=0; i<=reqs.size(); ++i) {
        if (i!=reqs.size()&&reqs[i].i==dep) {
            cout<<"OfflinePlanner: req issued by depot (node "<<dep<<")"<<endl;
            exit(-1);
        }
        for (int j=0; j<=reqs.size(); ++j) {
            if (i!=j) {
                if (i!=reqs.size()&&j!=reqs.size()&&reqs[i].i==reqs[j].i) {
                    cout<<"PotentialPlanner: dupl. request node "<<reqs[j].i
                            <<endl;
                    exit(-1);
                }
                int s=i==0?dep:reqs[i-1].i;
                int t=j==0?dep:reqs[j-1].i;
                tts[i][j]=dijk.cost(s, t)/data->instance().speed();
                if (j!=0)
                    tts[i][j]+=reqs[j-1].d;
            }
        }
    }
    return tts;
}

PlannedRoute OfflinePlanner::plannedRoute(const DVRPRoute& r) const {
    const auto& v=r.nodes;
    assert(v.size()>=3&&v[0]==0&&v.back()==0);
    Dijkstra dijk;
    vector<PlannedLink> links;
    const int dep=data->instance().depot();
    for (int j=1; j<v.size(); ++j) {
        int s=v[j-1]==0?dep:reqs.at(v.at(j-1)-1).i;
        int t=v[j]==0?dep:reqs.at(v.at(j)-1).i;
        Path p=dijk.solve(s, t);
        //Path& p=spaths[v[j-1]][v[j]];
        if (j==1)
            assert(p.links()[0].s==dep);
        else if (j==v.size()-1)
            assert(p.links().back().t==dep);
        for (const auto& l : p.links())
            links.emplace_back(l, vector<Request>());
        if (j!=v.size()-1)
            links.back().reqs.push_back(reqs[v[j]-1]);
    }
    return {links, 0};
}

vector<PlannedRoute> OfflinePlanner::plannedRoutes(
        const vector<DVRPRoute>& routes) const {
    vector<PlannedRoute> planned;
    for (const auto& r : routes)
        planned.push_back(plannedRoute(r));
    return planned;
}

