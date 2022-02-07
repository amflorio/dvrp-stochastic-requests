#ifndef DVRPSOLVER_H
#define DVRPSOLVER_H

#include <ilcplex/ilocplex.h>
#include <vector>
#include "DVRPLinearSolver.h"
#include "DVRPRoute.h"
#include "DVRPSolution.h"

ILOSTLBEGIN

class DVRPSolver {
    private:
        IloEnv env;
        IloModel model;
        IloCplex cplex;
        IloNumVarArray var;
        IloRangeArray con;
        IloObjective obj;
        const std::vector<std::vector<double>> costs;
        const int K;
        const double minlimit;
        const double maxlimit;
        std::vector<DVRPColumn> columns;
    public:
        DVRPSolver(std::vector<std::vector<double>>, int, double, double);
        ~DVRPSolver() {env.end();}
        std::vector<DVRPRoute> allRoutes() const;
        DVRPSolution solve();
};

#endif

