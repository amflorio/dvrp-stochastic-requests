#ifndef DVRPLINEARSOLVER_H
#define DVRPLINEARSOLVER_H

#include <ilcplex/ilocplex.h>
#include <memory>
#include "DVRPData.h"
#include "DVRPLinearSolution.h"
#include "DVRPRoute.h"

ILOSTLBEGIN

struct DVRPColumn {
    DVRPRoute route;
    bool included;
    DVRPColumn(DVRPRoute r) : route{r}, included{false} {}
};

class DVRPLinearSolver {
    private:
        const double pra_ratio=0.2;
        const int pra_iter=10;
        IloEnv env;
        IloModel model;
        IloCplex cplex;
        IloNumVarArray var;
        IloRangeArray con;
        IloObjective obj;
        std::shared_ptr<DVRPData> data;
        std::vector<DVRPColumn> cols;
        DVRPLinearSolution currSol;
        void printRoutes(const DVRPLinearSolution&) const;
        void solveExact();
        void solveHeuristic();
        void solvePRA();
        void solveRMP();
    public:
        static const int cols_exact=10;
        static const int cols_heur=10;
        DVRPLinearSolver(std::shared_ptr<DVRPData>);
        const std::vector<DVRPColumn>& columns() const {return cols;}
        DVRPLinearSolution solve();
};

#endif

