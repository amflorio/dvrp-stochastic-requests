#include <chrono>
#include "PolicySimulator.h"
#include "Stopwatch.h"
#include "TikZExporter.h"

using namespace std;
using hrclock=std::chrono::high_resolution_clock;

shared_ptr<Data> PolicySimulator::data;

void PolicySimulator::setIdleSaveRequests(vector<PlannedRoute>& vehics,
        double u, vector<Request>& served) const {
    // set to idle vehicles already back to the depot and save served reqs
    for (auto& r : vehics)
        if (r.finish()<=u) {
            auto reqs=r.requests();
            served.insert(served.end(), reqs.begin(), reqs.end());
            r.links={};
        }
}

SimResults PolicySimulator::simulate(const OfflinePlan& plan,
        const vector<Request>& dynreqs) const {
    assert(dynreqs.size()!=0);
    auto vehics=plan.routes;
    for (int k=vehics.size(); k<data->instance().vehicles(); ++k)
        vehics.emplace_back(vector<PlannedLink>(), 0);  // idle vehicles
    assert(vehics.size()==data->instance().vehicles());
    vector<Request> accepted;
    vector<Request> rejected;
    vector<Request> served;     // served reqs of vehicles already back to depot
    TikZExporter exp(data->instance(), "/Users/aflorio/Data/DVRPSR/TikZ/"
            +data->instance().code());
    exp.addState(0, {vehics, {-1,-1,-1}}, false, served, accepted.size(),
            0, 0);
    const double maxint=data->instance().period()/4000.0;
    vector<hrclock::duration> dectimes;
    for (const auto& req : dynreqs) {
        while (exp.timeLastState()<req.u-maxint) {
            double t=exp.timeLastState()+maxint;
            setIdleSaveRequests(vehics, t, served);
            exp.addState(t,{vehics, {-1,-1,-1}},false,served,accepted.size(),
                    SimResults::avgDecisionTime(dectimes),
                    SimResults::maxDecisionTime(dectimes));
        }
        cout<<"request arrived: "<<req<<endl;
        setIdleSaveRequests(vehics, req.u, served);
        Stopwatch sw;
        auto dec=policy->decide({vehics, req});
        sw.stop();
        dectimes.push_back(sw.elapsed());
        exp.addState(req.u, {vehics, req}, true, served, accepted.size(),
                SimResults::avgDecisionTime(dectimes),
                SimResults::maxDecisionTime(dectimes));
        cout<<"decision: "<<dec<<"  (time: "<<sw.elapsedSeconds()<<" s)"<<endl;
        validateDecision(dec, vehics, req);
        if (dec.accept) {
            if (vehics[dec.assign].links.size()==0)     // idle vehicle?
                vehics[dec.assign].start=req.u;
            vehics[dec.assign].links=dec.routing;
            cout<<"planned route ["<<dec.assign<<"] updated: "
                    <<vehics[dec.assign]<<endl;
            assert(vehics[dec.assign].feasible());
            accepted.push_back(req);
            cout<<"accepted requests: "<<accepted.size()<<endl;
        } else {
            rejected.push_back(req);
            cout<<"rejected requests: "<<rejected.size()<<endl;
        }
        if (policy->potential()!=-1)
            cout<<"projected total accepted: "
                    <<(int)(accepted.size()+policy->potential()+0.5)<<endl;
    }
    while (exp.timeLastState()<data->instance().period()-maxint) {
        double t=exp.timeLastState()+maxint;
        setIdleSaveRequests(vehics, t, served);
        exp.addState(t, {vehics, {-1,-1,-1}}, false, served, accepted.size(),
                SimResults::avgDecisionTime(dectimes),
                SimResults::maxDecisionTime(dectimes));
    }
    //cin.get();
    exp.setTexPolicy(policy->texString());
    //exp.exportAll();
    cout<<"not exporting .tex files!"<<endl;
    return {accepted, rejected, plan.requests(), dectimes};
}

void PolicySimulator::validateDecision(const Decision& dec,
        const vector<PlannedRoute>& vehics, const Request& r) {
    if (!dec.accept)
        return;
    if (dec.assign<0||dec.assign>=vehics.size()) {
        cout<<"PolicySimulator: accept decision, invalid assign"<<endl;
        exit(-1);
    }
    for (const auto& r : vehics)
        if (r.finish()>data->instance().period()) {
            cout<<"PolicySimulator: infeasible PlannedRoute"<<endl;
            exit(-1);
        }
    if (!vehics[dec.assign].links.empty()) {
        int idx=vehics[dec.assign].linkTraversed(r.u);
        for (int i=0; i<=idx; ++i)
            if (vehics[dec.assign].links[i].l.idx!=dec.routing[i].l.idx) {
                cout<<"PolicySimulator: accept decision, invalid routing"<<endl;
                exit(-1);
            }
    }
}

