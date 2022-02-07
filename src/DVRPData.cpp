#include <algorithm>
#include <cassert>
#include <iostream>
#include "DVRPData.h"

using namespace std;

DVRPData::DVRPData(vector<vector<double>> csts, int k, double lmt) :
        costs{move(csts)}, N{static_cast<int>(costs.size())}, K{k}, limit{lmt} {
    setupNgSets();
}

void DVRPData::setupNgSets() {
    assert(ngset_size<=N-1);
    ngindex=vector<vector<int>>(N, vector<int>(N, -1));
    ngnode.push_back(vector<int>());        // not defined for the depot
    for (int i=1; i<N; ++i) {
        ngnode.push_back(vector<int>());
        vector<pair<double, int>> v;
        for (int j=1; j<N; ++j)
            if (j!=i)
                v.emplace_back(costs[i][j], j);
        sort(v.begin(), v.end());
        for (int j=0; j<ngset_size; ++j) {
            ngnode[i].push_back(v[j].second);
            ngindex[i][v[j].second]=j;
        }
    }
    /*
    cout<<"ngnode:"<<endl;
    for (int i=0; i<ngnode.size(); ++i) {
        cout<<i<<": ";
        for (auto n : ngnode[i])
            cout<<n<<" ";
        cout<<endl;
    }
    cout<<"ngidx:"<<endl;
    for (int i=0; i<ngindex.size(); ++i) {
        cout<<i<<": ";
        for (auto idx : ngindex[i])
            cout<<idx<<" ";
        cout<<endl;
    }
    cin.get();
    */
}

