#include <chrono>
#include <iostream>
#include "SimResults.h"

using namespace std;

shared_ptr<Data> SimResults::data;

double SimResults::avgDecisionTime(const vector<hrclock::duration>& dectimes) {
    if (dectimes.empty())
        return 0;       // just to avoid 'nan's
    double tot=0;
    for (const auto& t : dectimes)
        tot+=chrono::duration_cast<chrono::milliseconds>(t).count();
    return (tot/dectimes.size())/1000;
}

vector<pair<int, int>> SimResults::cumulativeAcceptance() const {
    vector<pair<int, int>> vcum;
    // can be implemented more efficiently
    int U=(int)(data->instance().period()+0.5);
    for (int i=2; i<=U; i+=2) {
        int count=0;
        for (const auto& r : accepted)
            if (r.u<=i)
                count++;
            else
                break;
        vcum.emplace_back(i, count);
    }
    return vcum;
}

double SimResults::maxDecisionTime(const vector<hrclock::duration>& dectimes) {
    double mx=0;
    for (const auto& t : dectimes) {
        double ts=chrono::duration_cast<chrono::milliseconds>(t).count();
        if (ts>mx)
            mx=ts;
    }
    return mx/1000;
}

void SimResults::print() const {
    cout<<"Simulation results:"<<endl;
    cout<<"static requests: "<<static_.size()<<endl;
    cout<<"dynamic requests accepted: "<<accepted.size()<<endl;
    cout<<"dynamic requests rejected: "<<rejected.size()<<endl;
    cout<<"dynamic requests acceptance ratio: "<<(1.0*accepted.size())
            /(accepted.size()+rejected.size())<<endl;
    double totdurreqs=0;
    for (const auto& req : static_)
        totdurreqs+=req.d;
    for (const auto& req : accepted)
        totdurreqs+=req.d;
    cout<<"total duration of static and accepted dynamic requests: "<<totdurreqs
            <<endl;
    cout<<"(min \% over total service duration: "<<totdurreqs
            /(data->instance().vehicles()*data->instance().period())<<")"<<endl;
    cout<<"avg decision time: "<<avgDecisionTime(dectimes)<<" s"<<endl;
    cout<<"max decision time: "<<maxDecisionTime(dectimes)<<" s"<<endl;
    // uncomment below to output cumulative request acceptance over time
    /*
    cout<<"cumulative acceptance over time:"<<endl;
    auto vcum=cumulativeAcceptance();
    for (const auto& p : vcum)
        cout<<"cum: "<<p.first<<": "<<p.second<<endl;
    */
}

