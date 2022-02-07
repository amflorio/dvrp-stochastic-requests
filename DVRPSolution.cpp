#include <iostream>
#include "DVRPSolution.h"

using namespace std;

void DVRPSolution::print() const {
    cout<<"Solution value: "<<val<<endl;
    cout<<"Routes:"<<endl;
    for (const auto& r : routes)
        cout<<r<<endl;
}

