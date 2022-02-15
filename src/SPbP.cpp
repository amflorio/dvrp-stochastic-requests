#include <numeric>
#include "Dijkstra.h"
#include "Path.h"
#include "RouteReoptimizer.h"
#include "SPbP.h"

using namespace std;

Decision SPbP::decide(const State& s) const {
    auto decpots=decisionsPotentials(s);
    if (decpots.empty())
        return {false, 0, {}};
    pair<int, double> bestdec {-1, numeric_limits<double>::lowest()};
    for (int i=0; i<decpots.size(); ++i) {
        // return immediately if assigning to an idle vehicle
        if (s.vehics[get<0>(decpots[i]).assign].links.size()==0)
            return get<0>(decpots[i]);
        if (get<1>(decpots[i])>bestdec.second && get<1>(decpots[i])>0)
            bestdec={i, get<1>(decpots[i])};
    }
    if (bestdec.first==-1)
        return {false, 0, {}};
    Decision& best=get<0>(decpots[bestdec.first]);
    return best;
}

vector<Decision> SPbP::decisions(const State& s) const {
    vector<Decision> decs;
    auto decpots=decisionsPotentials(s);
    for (const auto& dec : decpots)
        decs.push_back(get<0>(dec));
    return decs;
}

vector<tuple<Decision, double>> SPbP::decisionsPotentials(const State& s) const{
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
                return decpots;
            } else
                break;  // not possible to serve with an idle vehicle
        }
    }
    // second: assign the request to not idle vehicles
    vector<vector<Request>> trjs;
    for (int i=0; i<Hpot; ++i)
        trjs.push_back(data->instance().sampleDynRequests(s.req.u));
    double totpot=0;
    for (int k=0; k<s.vehics.size(); ++k)
        if (s.vehics[k].links.size()!=0) {      // vehicle k not idle?
            double prej=s.vehics[k].potential(trjs, s.req.u);
            totpot+=prej;
            auto ci=cheapestInsertion(s.vehics[k], s.req);
            if (ci.second.size()!=0) {
                PlannedRoute r(ci.second, s.vehics[k].start);
                if (reopt) {
                    RouteReoptimizer reopter(r, s.req.u);
                    r=reopter.reoptimize();
                }
                if (r.feasible()) {
                    double pacc=r.potential(trjs, s.req.u);
                    //cout<<"pot(acc) ("<<k<<"): "<<pacc<<endl;
                    decpots.emplace_back(Decision(true, k, r.links), pacc-prej);
                }
            }
        }
    // adjust potentials based on mKP of reject decision
    double pmkp=PlannedRoute::potential(trjs, s.vehics, s.req.u);
    potential_=pmkp+0.5;    // +0.5 because accept/reject decision unknown here
    for (auto& dp : decpots) {
        get<1>(dp)/=totpot/pmkp;    // adjust for "intersecting" KPs
        get<1>(dp)+=1;              // accept decisions
    }
    return decpots;
}

std::string SPbP::texString() const {
    return "S-PbP$_{_\\textsf{\\scriptsize "+string(reopt?"R":"CI")+"}}$("
            +to_string(Hpot)+")";
}

