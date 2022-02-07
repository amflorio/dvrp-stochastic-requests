#include <ilcplex/ilocplex.h>
#include "DVRPLinearSolver.h"
#include "DVRPSolver.h"

using namespace std;

ILOSTLBEGIN

DVRPSolver::DVRPSolver(vector<vector<double>> csts, int k, double minlmt,
        double maxlmt) : model{env}, cplex{env}, var{env}, con{env},
        obj{IloMinimize(env)}, costs{move(csts)}, K{k}, minlimit{minlmt},
        maxlimit{maxlmt} {
    // number of vehicles constraint
    con.add(IloRange(env, -IloInfinity, K));
    // RHS of partitioning constraints
    for (int i=0; i<costs.size()-1; ++i)
        con.add(IloRange(env, 1, 1));
    model.add(obj);
    model.add(con);
    cplex.setParam(IloCplex::Threads, 1);
    //cplex.setOut(env.getNullStream());
    //cplex.setParam(IloCplex::NumParam::TiLim, 120);
    cplex.extract(model);
}

vector<DVRPRoute> DVRPSolver::allRoutes() const {
    vector<DVRPRoute> all;
    for (const auto& c : columns)
        all.push_back(c.route);
    return all;
}

DVRPSolution DVRPSolver::solve() {
    DVRPLinearSolver ls(make_shared<DVRPData>(costs, K, maxlimit));
    ls.solve();     // generate columns
    columns=ls.columns();
    try {
        columns.erase(remove_if(columns.begin(), columns.end(),
                [this](const DVRPColumn& c){return c.route.cost<minlimit;}),
                columns.end());
        for (const auto& c : columns) {
            IloNumVar v(env, 0, 1, ILOINT);
            var.add(v);
            obj.setLinearCoef(v, c.route.cost);
            con[0].setLinearCoef(v, 1);
            for (auto i : c.route.nodes)
                if (i!=0)
                    con[i].setLinearCoef(v, c.route.visits(i));
        }
        if (!cplex.solve()) {
            cout<<"DVRPSolver::solve(): can't solve model"<<endl;
            exit(-1);
        }
        IloNumArray vals(env);
        cplex.getValues(var, vals);
        vector<DVRPRoute> routes;
        for (int i=0; i<vals.getSize(); ++i)
            if (vals[i]>=0.5)
                routes.push_back(columns[i].route);
        vals.end();
        return {cplex.getObjValue(), routes};
    } catch (IloException& e) {
        cout<<"Concert exception caught: "<<e<<endl;
        exit(-1);
    } catch (...) {
        cout<<"Unknown exception caught"<<endl;
        exit(-1);
    }
}

