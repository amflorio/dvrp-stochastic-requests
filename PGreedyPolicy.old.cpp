#include <numeric>
#include "Dijkstra.h"
#include "Path.h"
#include "PGreedyPolicy.h"

using namespace std;

PGreedyPolicy::PGreedyPolicy() {
    if (Data::trajectories.empty())
        for (int i=0; i<100; ++i)
            Data::trajectories.push_back(data->instance().sampleDynRequests(0));
}

Decision PGreedyPolicy::decide(const State& s) const {
    // first: try to assign the request to an idle vehicle
    for (int k=0; k<s.vehics.size(); ++k) {
        if (s.vehics[k].links.size()==0) {      // vehicle k idle?
            // check if it is possible to serve request and return to the depot
            // before the end of the service period
            Dijkstra dijk;
            const int dep=data->instance().depot();
            Path to=dijk.solve(dep, s.req.i);
            Path from=dijk.solve(s.req.i, dep);
            if ((to.cost()+from.cost())/data->instance().speed()
                    +s.req.u+s.req.d<=data->instance().period()) {
                // it is possible to serve the request from the depot
                // create planned links
                vector<PlannedLink> newlinks;
                for (const auto& l : to.links())
                    newlinks.emplace_back(l, vector<Request>());
                newlinks.back().reqs.push_back(s.req);
                for (const auto& l : from.links())
                    newlinks.emplace_back(l, vector<Request>());
                // return accept decision
                return {true, k, newlinks};
            } else
                break;  // not possible to serve with an idle vehicle
        }
    }
    // second: try to assign the request to a not idle vehicle
    auto best=make_tuple(-1, vector<PlannedLink>(),
            numeric_limits<double>::lowest());
    for (int k=0; k<s.vehics.size(); ++k)
        if (s.vehics[k].links.size()!=0) {      // vehicle k not idle?
            auto cheapest=cheapestInsertion(s.vehics[k], s.req);
            if (cheapest.second.size()!=0) {
                PlannedRoute r(cheapest.second, s.vehics[k].start);
                if (r.feasible()) {
                    double potdiff=r.potential(s.req.u)
                            -s.vehics[k].potential(s.req.u);
                    if (potdiff>get<2>(best))
                        best={k, cheapest.second, potdiff};
                }
            }
        }
    if (get<0>(best)==-1)
        return {false, 0, {}};
    Decision dec(true, get<0>(best), get<1>(best));
    if (reopt)
        reoptimize(dec, s);
    return dec;
}

