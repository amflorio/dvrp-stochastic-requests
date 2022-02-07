#ifndef MPOTENTIALPLANNER_H
#define MPOTENTIALPLANNER_H

#include <vector>
#include <ilcplex/ilocplex.h>
#include "OfflinePlan.h"
#include "OfflinePlanner.h"
#include "Request.h"

ILOSTLBEGIN

class MPotentialPlanner : public OfflinePlanner {
    struct Solution {
        double val;
        std::vector<int> colidxs;
        std::vector<DVRPRoute> routes;
    };
    private:
        std::vector<DVRPRoute> routes;
        IloEnv env;
        IloModel model;
        IloCplex cplex;
        IloNumVarArray var;
        IloRangeArray con;
        IloObjective obj;
        std::vector<std::vector<Request>> trjs;
        std::vector<std::vector<DVRPRoute>> cols;
        Solution solve();
    public:
        MPotentialPlanner(std::vector<Request> rqs, std::vector<DVRPRoute> rts);
        ~MPotentialPlanner() {env.end();}
        OfflinePlan plan() override;
};

#endif

