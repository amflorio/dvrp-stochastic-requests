#ifndef DVRPDATA_H
#define DVRPDATA_H

#include <vector>

class DVRPData {
    private:
        void setupNgSets();
    public:
        const int ngset_size=6;
        std::vector<std::vector<int>> ngnode, ngindex;
        const std::vector<std::vector<double>> costs;
        const int N;
        const int K;
        double limit;
        DVRPData(std::vector<std::vector<double>> csts, int k, double lmt);
};

#endif

