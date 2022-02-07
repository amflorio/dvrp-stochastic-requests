#include <iostream>
#include "OfflinePlan.h"

using namespace std;

shared_ptr<Data> OfflinePlan::data;

void OfflinePlan::print() const {
    int k=0;
    double totdur=0;
    for (const auto& r : routes) {
        int reqs=0;
        double dur=0;
        for (const auto& l : r.links) {
            dur+=data->instance().links()[l.l.idx].d/data->instance().speed();
            for (const auto& r : l.reqs) {
                dur+=r.d;
                reqs++;
            }
        }
        cout<<"route "<<k++<<": duration: "<<dur<<" requests: "<<reqs<<endl;
        totdur+=dur;
    }
    cout<<"total duration ("<<k<<" routes): "<<totdur<<endl;
}

vector<Request> OfflinePlan::requests() const {
    vector<Request> reqs;
    for (const auto& r : routes)
        for (const auto& pl : r.links)
            reqs.insert(reqs.end(), pl.reqs.begin(), pl.reqs.end());
    return reqs;
}

