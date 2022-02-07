#ifndef ROUTEREOPTIMIZER_H
#define ROUTEREOPTIMIZER_H

#include <vector>
#include <ilcplex/ilocplex.h>
#include "PlannedRoute.h"

ILOSTLBEGIN

class RouteReoptimizer {
    private:
        IloEnv env;
        IloModel model;
        IloCplex cplex;
        IloNumVarArray var;
        IloRangeArray con;
        IloObjective obj;
        const PlannedRoute route;
        const double u;
        int N;
        std::vector<pair<int, int>> nodes;      // node id, idx in links
        std::vector<std::vector<double>> costs;
        void addConstraints();
        void addVarsObjCoeffs();
        PlannedRoute buildReoptimizedRoute(const std::vector<int>& tour) const;
    public:
        RouteReoptimizer(PlannedRoute r, double u_);
        static std::vector<int> getTour(
                const std::vector<std::vector<int>>& xijs);
        PlannedRoute reoptimize();
        ~RouteReoptimizer() {env.end();}
};

#endif

