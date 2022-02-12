#include <numeric>
#include "Dijkstra.h"
#include "Path.h"
#include "PFAPolicy.h"
#include "RouteReoptimizer.h"
#include "Stopwatch.h"

using namespace std;

PFAPolicy::PFAPolicy(OfflinePlan pln, int h) : plan{move(pln)}, H{h} {
    cout<<"PFAPolicy: reopt: "<<(reopt?"true":"false")<<endl;
    cout<<"PFAPolicy: tuning coefficient with "<<H<<" offline sims"<<endl;
    vector<vector<Request>> trjs;
    for (int i=0; i<H; ++i)
        trjs.push_back(data->instance().sampleDynRequests(0));
    // estimate Lambda from the trajectories
    const double Lambda=accumulate(trjs.begin(), trjs.end(), 0.0,
            [](double a, const vector<Request>& b){return a+b.size();})
            /(H*data->instance().period());
    cout<<"PFA: estimated Lambda: "<<Lambda<<endl;
    // setup initial state for offline simulations
    auto vehics=plan.routes;
    for (int k=vehics.size(); k<data->instance().vehicles(); ++k)
        vehics.emplace_back(vector<PlannedLink>(), 0);  // idle vehicles
    // variable step size search to find best value for gamma
    Stopwatch sw;
    double a=0;                         // interval begin
    double ss=0.02;                     // step size
    double last=0;                      // value of last reward
    pair<double, double> best {0,0};    // best gamma
    while (ss>0.001) {
        cout<<"interval: ["<<a<<", ...)   ;   step size: "<<ss<<endl;
        for (double s=a; true; s+=ss) {
            double totrew=simulateReward(vehics, trjs, s);
            cout<<"avg. reward gamma="<<s<<": "<<totrew/H<<"  "
                    <<(totrew>best.second?"***":"")<<endl;
            if (totrew>best.second) {
                best.first=s;
                best.second=totrew;
            }
            if (totrew>last)
                last=totrew;
            else {
                a=max(a, s-2*ss);
                ss/=2;
                last=0;
                break;
            }
        }
    }
    gamma=best.first;
    cout<<"PFA: parameter tuning: elapsed time: "<<sw.elapsedSeconds()<<" s"
            <<endl;
    cout<<"PFA: best gamma: "<<gamma<<endl;
}

Decision PFAPolicy::decide(const State& s) const {
    return decide(s, gamma);
}

Decision PFAPolicy::decide(const State& s, double g) const {
    auto decpots=decisionsPotentials(s, g);
    assert(!decpots.empty());
    pair<int, double> bestdec {-1, numeric_limits<double>::lowest()};
    for (int i=0; i<decpots.size(); ++i) {
        // return immediately if assigning to an idle vehicle
        if (s.vehics[decpots[i].first.assign].links.size()==0)
            return decpots[i].first;
        if (decpots[i].second>bestdec.second)
            bestdec={i, decpots[i].second};
    }
    assert(bestdec.first!=-1);
    return decpots[bestdec.first].first;
}

// TODO: set of candidate decisions should not come from the base policy
vector<Decision> PFAPolicy::decisions(const State& s) const {
    vector<Decision> decs;
    auto decpots=decisionsPotentials(s, gamma);
    for (const auto& dec : decpots)
        if (dec.first.accept)
            decs.push_back(dec.first);
    return decs;
}

vector<pair<Decision, double>> PFAPolicy::decisionsPotentials(const State& s,
        double g) const {
    vector<pair<Decision, double>> decpots;
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
                // potential ignored (=0) as this decision has precedence
                decpots.emplace_back(Decision(true, k, newlinks), 0);
            }
            break;  // no need to check other idle vehicles
        }
    }
    // second: reject the request
    vector<double> budgets(s.vehics.size(), 0);
    for (int k=0; k<budgets.size(); ++k)
        if (s.vehics[k].links.size()!=0)        // k not idle
            budgets[k]=data->instance().period()-s.vehics[k].finish();
    double totbudget=accumulate(budgets.begin(), budgets.end(), 0.0);
    decpots.emplace_back(Decision(false, 0, {}), g*totbudget);
    // third: assign the request to non-idle vehicles
    for (int k=0; k<s.vehics.size(); ++k)
        if (s.vehics[k].links.size()!=0) {      // k not idle
            auto ci=cheapestInsertion(s.vehics[k], s.req);
            if (ci.second.size()!=0) {
                PlannedRoute r(ci.second, s.vehics[k].start);
                if (reopt) {
                    RouteReoptimizer reopter(r, s.req.u);
                    r=reopter.reoptimize();
                }
                if (r.feasible()) {
                    decpots.emplace_back(Decision(true, k, r.links), 1+
                            g*(data->instance().period()-r.finish()
                            +totbudget-budgets[k]));
                }
            }
        }
    return decpots;
}

int PFAPolicy::simulateReward(const vector<PlannedRoute>& routes,
        const vector<vector<Request>>& trjs, double g) {
    static vector<pair<double, int>> cache;
    for (const auto& c : cache)
        if (abs(g-c.first)<1e-10)
            return c.second;
    int totrew=0;
    for (const auto& trj : trjs) {
        auto routescpy=routes;
        for (const auto& req : trj) {
            for (auto& r : routescpy)
                if (r.finish()<=req.u)
                    r.links={};
            auto dec=decide({routescpy, req}, g);
            if (dec.accept) {
                if (routescpy[dec.assign].links.size()==0)      // idle?
                    routescpy[dec.assign].start=req.u;
                routescpy[dec.assign].links=dec.routing;
                totrew++;
            }
        }
    }
    cache.push_back(pair<double, int>(g, totrew));
    return totrew;
}

string PFAPolicy::texString() const {
    return string("PFA$_{_\\textsf{\\scriptsize ")+(reopt?"R":"CI")+"}}$";
}

