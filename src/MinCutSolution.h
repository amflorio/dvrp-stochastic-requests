#ifndef MINCUTSOLUTION_H
#define MINCUTSOLUTION_H

#include <vector>

class MinCutSolution {
    private:
        std::vector<int> mincut;
        double val;
    public:
        MinCutSolution(const std::vector<int>& cut, double value) : mincut{cut},
                val{value} {}
        std::vector<int> minCut() {return mincut;}
        double value() {return val;}
};

#endif

