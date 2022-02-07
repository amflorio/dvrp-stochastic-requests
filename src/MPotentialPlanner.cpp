#include <ilcplex/ilocplex.h>
#include "MPotentialPlanner.h"
#include "PlannedRoute.h"

using std::vector;

ILOSTLBEGIN

MPotentialPlanner::MPotentialPlanner(vector<Request> rqs, vector<DVRPRoute> rts)
        : OfflinePlanner{move(rqs)}, routes{move(rts)}, model(env), cplex(env),
        var(env), con(env), obj(IloMaximize(env)) {
    routes.erase(remove_if(routes.begin(), routes.end(), [](const DVRPRoute& r)
            {return !r.elementary()||r.cost>data->instance().period();}),
            routes.end());
    // trajectories for potential evaluation
    for (int i=0; i<50; ++i)
        trjs.push_back(data->instance().sampleDynRequests(0));
    if (trjs.size()<50)
        cout<<"WARNING: using only "<<trjs.size()<<" trajectories"<<endl;
    // create base model
    int K=data->instance().vehicles();
    con.add(IloRange(env, -IloInfinity, K));
    for (int i=0; i<reqs.size(); ++i)
        con.add(IloRange(env, 1, 1));
    model.add(obj);
    model.add(con);
    cplex.setParam(IloCplex::Threads, 1);
    cplex.setOut(env.getNullStream());
    //cplex.setParam(IloCplex::NumParam::TiLim, 120);
    cplex.extract(model);
    for (const auto& r : routes) {
        cols.push_back({r});
        IloNumVar v(env, 0, 1, ILOINT);
        var.add(v);
        obj.setLinearCoef(v, plannedRoute(r).potential(trjs, 0));
        con[0].setLinearCoef(v, 1);
        for (int i : r.nodes)
            if (i!=0)
                con[i].setLinearCoef(v, r.visits(i));
        if (var.getSize()%100==0)
            cout<<var.getSize()<<"/"<<routes.size()<<" variables added ..."
                    <<endl;
    }
}

OfflinePlan MPotentialPlanner::plan() {
    int it=1;       // iteration counter
    while (true) {
        auto currsol=solve();
        // "remove" all columns of currsol
        for (int i : currsol.colidxs)
            obj.setLinearCoef(var[i], 0);
        // compute potential of the set and check convergence
        double setpot=PlannedRoute::potential(trjs,
                plannedRoutes(currsol.routes), 0);
        cout<<(it++)<<": current solution: sum of potentials: "<<currsol.val
                <<" ; potential of the set: "<<setpot<<endl;
        assert(setpot<=currsol.val+1e-4);
        if (abs(currsol.val-setpot)<1e-3)      // converged?
            return {plannedRoutes(currsol.routes)};
        // add one column representing the set/solution
        cols.push_back({currsol.routes});
        IloNumVar v(env, 0, 1, ILOINT);
        var.add(v);
        obj.setLinearCoef(v, setpot);
        con[0].setLinearCoef(v, currsol.routes.size());
        for (int i=1; i<=reqs.size(); ++i)
            con[i].setLinearCoef(v, 1);
        assert(cols.size()==var.getSize());
    }
}

/*
OfflinePlan MPotentialPlanner::plan() {
    return {plannedRoutes(solveMaxPotential(reqs.size()))};
}
*/

MPotentialPlanner::Solution MPotentialPlanner::solve() {
    if (!cplex.solve()) {
        cout<<"MPotentialPlanner: can't solve model"<<endl;
        exit(-1);
    }
    IloNumArray vals(env);
    cplex.getValues(var, vals);
    Solution sol;
    sol.val=cplex.getObjValue();
    for (int i=0; i<vals.getSize(); ++i)
        if (vals[i]>=0.5) {
            sol.colidxs.push_back(i);
            sol.routes.insert(sol.routes.end(), cols[i].begin(), cols[i].end());
        }
    vals.end();
    return sol;
}

