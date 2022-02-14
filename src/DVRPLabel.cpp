#include <cassert>
#include "DVRPLabel.h"

using namespace std;

shared_ptr<DVRPData> DVRPLabel::data;

DVRPLabel::DVRPLabel(vector<int> nds, double c, double dls) :
        visits(data->N, 0), nodes{move(nds)}, cost{c}, duals{dls},
        ngset(data->ngset_size, 0) {
    assert(nodes.size()==2);        // because not updating ngset
    for (auto i : nodes)
        visits[i]++;
}

bool DVRPLabel::dominates(const DVRPLabel& l) const {
    if (cost>l.cost)
        return false;
    return (ngbin&(~l.ngbin))==0;
    /*
    for (int k=0; k<ngset.size(); ++k)
        if (ngset[k]>l.ngset[k])
            return false;
    return true;
    */
}

DVRPLabel DVRPLabel::extend(int i, double c, double dl) const {
    DVRPLabel ext(*this);
    assert(!ext.dominated&&!ext.extended);
    ext.visits[i]++;
    ext.nodes.push_back(i);
    ext.cost+=c;
    ext.duals+=dl;
    // update ngset
    ext.ngbin=0;
    for (int k=0; k<ext.ngset.size(); ++k) {
        int n=data->ngnode[i][k];
        if (n==nodes.back()) {
            ext.ngset[k]=1;
            ext.ngbin+=1<<k;
        } else {
            int idx=data->ngindex[nodes.back()][n];
            if (idx==-1 || ngset[idx]==0)
                ext.ngset[k]=0;
            else {
                ext.ngset[k]=1;
                ext.ngbin+=1<<k;
            }
        }
    }
    return ext;
}

bool DVRPLabel::ngForbid(int j) const {
    int idx=data->ngindex[nodes.back()][j];
    if (idx==-1 || ngset[idx]==0)
        return false;
    return true;
}

void DVRPLabel::print() const {
    cout<<"printing label ..."<<endl;
    for (int i=0; i<visits.size(); ++i)
        if (visits[i])
            cout<<"visits["<<i<<"]: "<<visits[i]<<endl;
    for (int i=0; i<nodes.size(); ++i)
        cout<<nodes[i]<<" ";
    cout<<endl;
    cout<<"cost: "<<cost<<endl;
    cout<<"duals: "<<duals<<endl;
}

void DVRPLabel::updateCost(const double kDual, const vector<double>& pttDuals) {
    duals=kDual;
    for (auto i : nodes)
        if (i!=0)
            duals+=pttDuals[i];
}

