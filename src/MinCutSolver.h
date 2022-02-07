#ifndef MINCUTSOLVER_H
#define MINCUTSOLVER_H

#include <vector>
#include "MinCutSolution.h"

class MinCutSolver {
    private:
        std::vector<std::vector<double>> network;
    public:
        MinCutSolver(const std::vector<std::vector<double>>& xijs) :
                network{xijs} {}
        MinCutSolution solve();
};

#endif

