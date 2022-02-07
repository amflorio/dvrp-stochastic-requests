#include <ilcplex/ilocplex.h>
#include "PotentialPlanner.h"

using std::vector;

ILOSTLBEGIN

PotentialPlanner::PotentialPlanner(vector<Request> rqs, vector<DVRPRoute> rts)
        : OfflinePlanner{move(rqs)}, routes{move(rts)} {
    routes.erase(remove_if(routes.begin(), routes.end(), [](const DVRPRoute& r)
            {return !r.elementary()||r.cost>data->instance().period();}),
            routes.end());
}

OfflinePlan PotentialPlanner::plan() {
    return {plannedRoutes(solveMaxPotential(reqs.size()))};
}

vector<DVRPRoute> PotentialPlanner::solveMaxPotential(int N) const {
    IloEnv env;
    IloModel model(env);
    IloCplex cplex(env);
    IloNumVarArray var(env);
    IloRangeArray con(env);
    IloObjective obj(IloMaximize(env));
    int K=data->instance().vehicles();
    con.add(IloRange(env, -IloInfinity, K));
    for (int i=0; i<N; ++i)
        con.add(IloRange(env, 1, 1));
    model.add(obj);
    model.add(con);
    cplex.setParam(IloCplex::Threads, 1);
    //cplex.setOut(env.getNullStream());
    //cplex.setParam(IloCplex::NumParam::TiLim, 120);
    cplex.extract(model);
    vector<vector<Request>> trjs;
    for (int i=0; i<50; ++i)
        trjs.push_back(data->instance().sampleDynRequests(0));
    for (const auto& r : routes) {
        IloNumVar v(env, 0, 1, ILOINT);
        var.add(v);
        obj.setLinearCoef(v, plannedRoute(r).potential(trjs, 0));
        con[0].setLinearCoef(v, 1);
        for (int i : r.nodes)
            if (i!=0)
                con[i].setLinearCoef(v, r.visits(i));
        //con[N+1].setLinearCoef(v, r.cost);
        if (var.getSize()%100==0)
            cout<<var.getSize()<<"/"<<routes.size()<<" variables added ..."
                    <<endl;
    }
    if (!cplex.solve()) {
        cout<<"PotentialPlanner: can't solve model"<<endl;
        exit(-1);
    }
    IloNumArray vals(env);
    cplex.getValues(var, vals);
    vector<DVRPRoute> routessol;
    for (int i=0; i<vals.getSize(); ++i)
        if (vals[i]>=0.5)
            routessol.push_back(routes[i]);
    var.end();
    con.end();
    vals.end();
    env.end();
    return routessol;
}

