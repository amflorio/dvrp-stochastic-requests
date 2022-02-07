#include "Path.h"

using namespace std;

shared_ptr<Data> Path::data;

Path::Path(vector<Link> links, double cost) : links_{move(links)}, c{cost} {
    //assert(data!=nullptr);
    assert(links_.size()!=0);
    s=links_[0].s;
}

ostream& operator<<(ostream& os, const Path& p) {
    if (p.links_.size()==0) {
        os<<"["<<p.s<<"]";
        return os;
    }
    os<<"["<<p.links_[0].s;
    for (int i=0; i<p.links_.size(); ++i)
        os<<"-"<<p.links_[i].t;
    os<<"] c="<<p.c;
    return os;
}

