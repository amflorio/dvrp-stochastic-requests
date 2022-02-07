#include <limits>
#include <vector>
#include "MaxFlowSolution.h"
#include "MaxFlowSolver.h"
#include "MinCutSolver.h"

using namespace std;

MinCutSolution MinCutSolver::solve() {
    int s=0;
    double mincutvalue=numeric_limits<double>::infinity();
    vector<int> mincut;
    for (int t=1; t<network.size(); ++t) {
        {
            MaxFlowSolver maxflow(s, t, network);
            MaxFlowSolution sol=maxflow.solve();
            if (sol.value()<mincutvalue) {
                mincutvalue=sol.value();
                mincut=sol.minCut();
            }
        }
        {
            MaxFlowSolver maxflow(t, s, network);
            MaxFlowSolution sol=maxflow.solve();
            if (sol.value()<mincutvalue) {
                mincutvalue=sol.value();
                mincut=sol.minCut();
            }
        }
    }
    return MinCutSolution(mincut, mincutvalue);
}

