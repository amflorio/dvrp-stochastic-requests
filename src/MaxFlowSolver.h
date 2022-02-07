#ifndef MAXFLOWSOLVER_H
#define MAXFLOWSOLVER_H

#include "MaxFlowSolution.h"

class MaxFlowSolver {
    private:
        int s;
        int t;
        std::vector<std::vector<double>> network;
        double augmentFlow(const std::vector<int>& path,
                std::vector<std::vector<double>>& residual);
        std::vector<int> findAugmentingPath(
                const std::vector<std::vector<double>>& residual);
        std::vector<int> findCut(
                const std::vector<std::vector<double>>& residual);
    public:
        MaxFlowSolver(int _s, int _t,
                const std::vector<std::vector<double>>& _network) : s{_s},
                t{_t}, network{_network} {}
        MaxFlowSolution solve();
};

#endif

