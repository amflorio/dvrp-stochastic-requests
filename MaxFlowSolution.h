#ifndef MAXFLOWSOLUTION_H
#define MAXFLOWSOLUTION_H

#include <vector>

class MaxFlowSolution {
    private:
        double maxflow;
        std::vector<int> mincut;
    public:
        MaxFlowSolution(double flow, const std::vector<int>& cut) :
                maxflow{flow}, mincut{cut} {}
        std::vector<int> minCut() {return mincut;}
        double value() {return maxflow;}
};

#endif

