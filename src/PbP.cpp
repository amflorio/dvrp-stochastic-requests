#include <numeric>
#include "Dijkstra.h"
#include "Path.h"
#include "PbP.h"
#include "RouteReoptimizer.h"

using namespace std;

Decision PbP::decide(const State& s) const {
    auto decpots=decisionsPotentials(s);
    assert(!decpots.empty());
    pair<int, double> bestdec {-1, numeric_limits<double>::lowest()};
    for (int i=0; i<decpots.size(); ++i) {
        // return immediately if assigning to an idle vehicle
        if (s.vehics[get<0>(decpots[i]).assign].links.size()==0) {
            potential_=-1;
            return get<0>(decpots[i]);
        }
        if (get<1>(decpots[i])>bestdec.second)
            bestdec={i, get<1>(decpots[i])};
    }
    Decision& best=get<0>(decpots[bestdec.first]);
    potential_=bestdec.second;
    if (best.accept)
        potential_-=1;
    return best;
}

vector<Decision> PbP::decisions(const State& s) const {
    vector<Decision> decs;
    auto decpots=decisionsPotentials(s);
    for (const auto& dec : decpots)
        decs.push_back(get<0>(dec));
    return decs;
}

vector<tuple<Decision, double>> PbP::decisionsPotentials(const State& s) const {
    vector<tuple<Decision, double>> decpots;
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
                decpots.emplace_back(Decision(true, k, newlinks), 0);
            } else
                break;  // not possible to serve with an idle vehicle
        }
    }
    // second: assign the request to not idle vehicles
    vector<vector<Request>> trjs;
    for (int i=0; i<Hpot; ++i)
        trjs.push_back(data->instance().sampleDynRequests(s.req.u));
    // potential of `reject' decision
    double prej;
    decpots.emplace_back(Decision(false, 0, {}),
            prej=PlannedRoute::potential(trjs, s.vehics, s.req.u));
    //cout<<"pot(rej): "<<prej<<endl;
    for (int k=0; k<s.vehics.size(); ++k)
        if (s.vehics[k].links.size()!=0) {      // vehicle k not idle?
            auto ci=cheapestInsertion(s.vehics[k], s.req);
            if (ci.second.size()!=0) {
                PlannedRoute r(ci.second, s.vehics[k].start);
                if (reopt) {
                    RouteReoptimizer reopter(r, s.req.u);
                    r=reopter.reoptimize();
                }
                if (r.feasible()) {
                    auto vehicscpy=s.vehics;
                    vehicscpy[k]=r;
                    double pacc;
                    decpots.emplace_back(Decision(true, k, r.links), pacc=1+
                            PlannedRoute::potential(trjs, vehicscpy, s.req.u));
                    //cout<<"pot(acc) ("<<k<<"): "<<pacc<<endl;
                }
            }
        }
    return decpots;
}

std::string PbP::texString() const {
    return "PbP$_{_\\textsf{\\scriptsize "+string((reopt?"R":"CI"))+"}}$("
            +to_string(Hpot)+")";
}

