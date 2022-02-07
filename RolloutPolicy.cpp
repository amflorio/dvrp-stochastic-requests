#include <numeric>
#include "Dijkstra.h"
#include "Path.h"
#include "PolicySimulator.h"
#include "RolloutPolicy.h"

using namespace std;

Decision RolloutPolicy::decide(const State& s) const {
    // TODO: set of candidate decisions should not come from the base policy
    // TODO: currently, rollout may _not_ assign an accepted req to idle vehicle
    auto decisions=basepol->decisions(s);
    if (reopt)
        for (auto& dec : decisions)
            if (dec.accept) {
                cout<<"reoptimizing decision: "<<dec<<endl;
                assert(dec.routing.size()!=0);
                reoptimize(dec, s);
            }
    if (decisions.size()==0)
        return {false, 0, {}};      // only option is reject
    vector<vector<Request>> trjs;
    for (int i=0; i<H; ++i) // gen and store H trajectories from instant s.req.u
        trjs.push_back(data->instance().sampleDynRequests(s.req.u));
    auto best=make_tuple(Decision(false, 0, {}), rollout(s.vehics, trjs));
    cout<<"rollout: decision "<<get<0>(best)<<" ; reward "<<get<1>(best)<<endl;
    for (const auto& dec : decisions) {
        assert(dec.accept);
        auto newroutes=s.vehics;
        if (s.vehics[dec.assign].links.empty())
            newroutes[dec.assign].start=s.req.u;
        newroutes[dec.assign].links=dec.routing;
        int reward=H+rollout(newroutes, trjs);
        cout<<"rollout: decision "<<dec<<" ; reward "<<reward<<endl;
        if (reward>get<1>(best))
            best={dec, reward};
    }
    return get<0>(best);
}

int RolloutPolicy::rollout(const vector<PlannedRoute>& simRoutes,
        const vector<vector<Request>>& simSamples) const {
    int accepted=0;
    assert(H==simSamples.size());
    for (int h=0; h<H; ++h) {
        auto routes=simRoutes;
        if (h%7==0||h==H-1)
            cout<<"\th = "<<h<<endl;
        for (const auto& i : simSamples[h]) {
            assert(routes.size()==data->instance().vehicles());
            for (auto& r : routes) {
                if (r.finish()<=i.u)
                    r.links={};   
            }
            Decision simdec=basepol->decide({routes, i});
            if (simdec.accept) {
                if (routes[simdec.assign].links.size()==0)   // idle?
                    routes[simdec.assign].start=i.u;
                routes[simdec.assign].links=simdec.routing;
                //assert(routes[simdec.assign].feasible());// TODO: expensive?
                accepted++;
            }
        }
    }
    return accepted;
}

std::string RolloutPolicy::texString() const {
    return "R$_{_\\textsf{\\scriptsize "+string(reopt?"R":"CI")+"}}$-"
            +basepol->texString()+"("+to_string(H)+")";
}

