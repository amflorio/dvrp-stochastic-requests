#include <iostream>
#include <set>
#include "DVRPRoute.h"

using namespace std;

bool DVRPRoute::elementary() const {
    set<int> nodeset;
    for (int i : nodes)
        if (i!=0 && !nodeset.insert(i).second)
            return false;
    return true;
}

int DVRPRoute::visits(int i) const {
    int k=0;
    for (auto j : nodes)
        if (j==i)
            k++;
    return k;
}

ostream& operator<<(ostream& os, const DVRPRoute& r) {
    os<<"["<<r.nodes[0];
    for (int i=1; i<r.nodes.size(); ++i)
        os<<" "<<r.nodes[i];
    os<<"] n="<<(r.nodes.size()-2)<<" c="<<r.cost<<" rc="<<r.redcost;
    return os;
}

