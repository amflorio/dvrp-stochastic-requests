#include "DVRPLinearSolver.h"
#include "DVRPPricing.h"

using namespace std;

ILOSTLBEGIN

DVRPLinearSolver::DVRPLinearSolver(shared_ptr<DVRPData> d) : model{env},
        cplex{env}, var{env}, con{env}, obj{IloMinimize(env)}, data{d},
        currSol(numeric_limits<double>::infinity(),
        vector<pair<double, DVRPRoute>>(), 0, vector<double>()) {
    DVRPLabel::data=data;
    // number of vehicles constraint
    con.add(IloRange(env, -IloInfinity, data->K));
    // RHS of partitioning constraints
    for (int i=1; i<data->N; ++i)
        con.add(IloRange(env, 1, 1));
    model.add(obj);
    model.add(con);
    cplex.setParam(IloCplex::Threads, 1);
    cplex.setOut(env.getNullStream());
    cplex.extract(model);
}

void DVRPLinearSolver::printRoutes(const DVRPLinearSolution& sol) const {
    for (const auto& r : sol.routes)
        cout<<r.first<<" "<<r.second<<endl;
}

DVRPLinearSolution DVRPLinearSolver::solve() {
    solvePRA();
    solveHeuristic();
    solveExact();
    return currSol;
}

void DVRPLinearSolver::solveExact() {
    while (true) {
        solveRMP();
        cout<<"e: current RMP sol value: "<<currSol.val<<endl;
        printRoutes(currSol);
        cout<<"e: solving pricing from curr RMP solution"<<endl;
        DVRPPricing pricing(data, currSol);
        auto routes=pricing.solveExact();
        if (routes.size()==0) {
            cout<<"e: no cols with neg rc found"<<endl;
            return;
        }
        cout<<"e: found "<<routes.size()<<" cols with neg rc:"<<endl;
        for (auto& r : routes) {
            cout<<r<<endl;
            cols.emplace_back(move(r));
        }
    }
}

void DVRPLinearSolver::solveHeuristic() {
    int fails=1;        // fails==2 and we finish the heuristic
    while (true) {
        solveRMP();
        cout<<"h: current RMP sol value: "<<currSol.val<<endl;
        printRoutes(currSol);
        cout<<"h: solving pricing from curr RMP solution"<<endl;
        DVRPPricing pricing(data, currSol);
        auto routes=pricing.solveHeuristic();
        cout<<"h: found "<<routes.size()<<" cols with neg rc:"<<endl;
        for (auto& r : routes) {
            cout<<r<<endl;
            cols.emplace_back(move(r));
        }
        if (routes.size()<cols_heur) {
            fails++;
            if (fails==2)
                return;
        } else
            fails=0;
    }
}

void DVRPLinearSolver::solvePRA() {
    // depot-i-depot routes
    for (int i=1; i<data->N; ++i)
        cols.emplace_back(
                DVRPRoute(data->costs[0][i]+data->costs[i][0],{0,i,0},0));
    // giant (infeasible) route
    double cgiant=0;
    vector<int> vgiant {0};
    for (int i=1; i<data->N; ++i) {
        cgiant+=data->costs[i-1][i];
        vgiant.push_back(i);
    }
    cgiant+=data->costs[vgiant.back()][0];
    vgiant.push_back(0);
    cols.emplace_back(DVRPRoute(cgiant, vgiant, 0));
    const double limit=data->limit;
    double praLimit=limit*pra_ratio;
    int inc=(limit-praLimit)/pra_iter;
    while (praLimit<limit) {
        data->limit=praLimit;
        while (true) {
            solveRMP();
            DVRPPricing pricing(data, currSol);
            auto routes=pricing.solveHeuristic();
            for (auto& r : routes) {
                cout<<r<<endl;
                cols.emplace_back(move(r));
            }
            if (routes.size()<cols_heur)
                break;
        }
        solveRMP();
        cout<<"PRA: done for limit="<<praLimit<<"\tval="<<currSol.val<<endl;
        praLimit+=inc;
    }
    data->limit=limit;
    cout<<"PRA finished"<<endl;
}

void DVRPLinearSolver::solveRMP() {
    try {
        for (auto& c : cols) {
            if (!c.included) {
                IloNumVar v(env, 0, IloInfinity);
                var.add(v);
                obj.setLinearCoef(v, c.route.cost);
                con[0].setLinearCoef(v, 1);
                for (auto i : c.route.nodes)
                    if (i!=0)
                        con[i].setLinearCoef(v, c.route.visits(i));
                c.included=true;
            }
        }
        //cplex.exportModel("lp.lp");
        if (!cplex.solve()) {
            cout<<"DVRPLinearSolver::solveRMP(): can't solve model"<<endl;
            exit(-1);
        } 
        IloNumArray vals(env);
        cplex.getValues(var, vals);
        vector<pair<double, DVRPRoute>> routes;
        for (int i=0; i<vals.getSize(); ++i)
            if (vals[i]>=1e-5)
                routes.emplace_back(vals[i], cols[i].route);
        vals.end();
        IloNumArray duals(env);
        cplex.getDuals(duals, con);
        assert(duals.getSize()==data->N);
        vector<double> dls(data->N, 0);
        for (int i=1; i<duals.getSize(); ++i)
            dls[i]=duals[i];
        currSol=DVRPLinearSolution(cplex.getObjValue(), routes, duals[0], dls);
        duals.end();
    } catch (IloException& e) {
        cout<<"Concert exception caught: "<<e<<endl;
        exit(-1);
    } catch (...) {
        cout<<"Unknown exception caught"<<endl;
        exit(-1);
    }
}

