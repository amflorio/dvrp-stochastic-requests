#include <numeric>
#include "Dijkstra.h"
#include "GreedyPolicy.h"
#include "Path.h"
#include "RouteReoptimizer.h"

using namespace std;

Decision GreedyPolicy::decide(const State& s) const {
    auto deccosts=decisionsCosts(s);
    pair<int, double> bestdec {-1, numeric_limits<double>::max()};
    for (int i=0; i<deccosts.size(); ++i) {
        // return immediately if assigning to an idle vehicle
        if (s.vehics[deccosts[i].first.assign].links.size()==0)
            return deccosts[i].first;
        if (deccosts[i].second<bestdec.second)
            bestdec={i, deccosts[i].second};
    }
    if (bestdec.first==-1)
        return {false, 0, {}};
    Decision& best=deccosts[bestdec.first].first;
    return best;
}

vector<Decision> GreedyPolicy::decisions(const State& s) const {
    vector<Decision> decs;
    auto deccosts=decisionsCosts(s);
    for (const auto& dec : deccosts)
        decs.push_back(dec.first);
    return decs;
}

vector<pair<Decision, double>> GreedyPolicy::decisionsCosts(const State& s)
        const {
    vector<pair<Decision, double>> deccosts;
    // first: assign the request to an idle vehicle
    for (int k=0; k<s.vehics.size(); ++k) {
        if (s.vehics[k].links.size()==0) {      // vehicle k idle?
            // check if it is possible to serve request and return to the depot
            Dijkstra dijk;
            const int dep=data->instance().depot();
            Path to=dijk.solve(dep, s.req.i);
            Path from=dijk.solve(s.req.i, dep);
            double dur=(to.cost()+from.cost())/data->instance().speed()+s.req.d;
            if (s.req.u+dur<=data->instance().period()) {
                // it is possible, create planned links
                vector<PlannedLink> newlinks;
                for (const auto& l : to.links())
                    newlinks.emplace_back(l, vector<Request>());
                newlinks.back().reqs.push_back(s.req);
                for (const auto& l : from.links())
                    newlinks.emplace_back(l, vector<Request>());
                deccosts.emplace_back(Decision(true, k, newlinks), 0);
            }
            break;
        }
    }
    // second: assign the request to not idle vehicles
    for (int k=0; k<s.vehics.size(); ++k)
        if (s.vehics[k].links.size()!=0) {      // vehicle k not idle?
            auto ci=cheapestInsertion(s.vehics[k], s.req);
            if (ci.second.size()!=0) {
                PlannedRoute r(ci.second, s.vehics[k].start);
                if (reopt) {
                    RouteReoptimizer reopter(r, s.req.u);
                    r=reopter.reoptimize();
                }
                if (r.feasible())
                    deccosts.emplace_back(Decision(true, k, r.links),
                            r.finish()-s.vehics[k].finish());
            }
        }
    return deccosts;
}

std::string GreedyPolicy::texString() const {
    return string("GP$_{_\\textsf{\\scriptsize ")+(reopt?"R":"CI")+"}}$";
}

