#include <ilcplex/ilocplex.h>
#include <vector>
#include "Dijkstra.h"
#include "MinCutSolver.h"
#include "RouteReoptimizer.h"

using namespace std;

ILOSTLBEGIN

ILOLAZYCONSTRAINTCALLBACK2(TSPSolverSECLC, IloNumVarArray, xijs, int, N) {
    IloNumArray valscplex(getEnv());
    getValues(valscplex, xijs);
    vector<vector<double>> vals(N, vector<double>(N, 0));
    for (int k=0, i=0; i<N; ++i)
        for (int j=0; j<N; ++j, ++k)
            if (i!=j)
                vals[i][j]=valscplex[k];
    valscplex.end();
    // subtour elimination constraints (fractional) (global cuts)
    const double CUTTOL=1e-3;
    MinCutSolver mincut(vals);
    MinCutSolution sol=mincut.solve();
    if (sol.value()<1-CUTTOL) {
        assert(sol.minCut().size()==N);
        IloNumVarArray vars(getEnv());
        IloNumArray coeffs(getEnv());
        for (int i=0; i<N; ++i) {
            if (sol.minCut()[i]) {      // i is in the cut
                for (int j=0; j<N; ++j) {
                    if (i!=j && !sol.minCut()[j]) { // j is NOT in the cut
                        vars.add(xijs[i*N+j]);
                        coeffs.add(1);
                    }
                }
            }
        }
        assert(vars.getSize()!=0);
        add(IloRange(getEnv(), 1, IloScalProd(vars, coeffs), IloInfinity));
        vars.end();
        coeffs.end();
    }
}

ILOUSERCUTCALLBACK2(TSPSolverSECUC, IloNumVarArray, xijs, int, N) {
    IloNumArray valscplex(getEnv());
    getValues(valscplex, xijs);
    vector<vector<double>> vals(N, vector<double>(N, 0));
    for (int k=0, i=0; i<N; ++i)
        for (int j=0; j<N; ++j, ++k)
            if (i!=j)
                vals[i][j]=valscplex[k];
    valscplex.end();
    // subtour elimination constraints (fractional) (global cuts)
    const double CUTTOL=1e-3;
    MinCutSolver mincut(vals);
    MinCutSolution sol=mincut.solve();
    if (sol.value()<1-CUTTOL) {
        assert(sol.minCut().size()==N);
        IloNumVarArray vars(getEnv());
        IloNumArray coeffs(getEnv());
        for (int i=0; i<N; ++i) {
            if (sol.minCut()[i]) {      // i is in the cut
                for (int j=0; j<N; ++j) {
                    if (i!=j && !sol.minCut()[j]) { // j is NOT in the cut
                        vars.add(xijs[i*N+j]);
                        coeffs.add(1);
                    }
                }
            }
        }
        assert(vars.getSize()!=0);
        add(IloRange(getEnv(), 1, IloScalProd(vars, coeffs), IloInfinity));
        vars.end();
        coeffs.end();
    }
}

RouteReoptimizer::RouteReoptimizer(PlannedRoute r, double u_) : model{env},
        cplex{env}, var{env}, con{env}, obj{IloMinimize(env)}, route{move(r)},
        u{u_} {
    assert(route.links.size()!=0);
    // determine the nodes
    int next=route.linkTraversed(u);
    assert(next!=route.links.size()-1);
    int curr;
    do {
        curr=next+1;
        nodes.push_back({route.links.at(curr).l.s, curr-1});
        for (next=curr; route.links.at(next).reqs.size()==0
                &&next<route.links.size()-1; ++next);
    } while (next<route.links.size()-1);
    nodes.push_back({route.links.back().l.t, route.links.size()-1});
    /*
    cout<<"planned route (before reopt): "<<route<<endl;
    cout<<"nodes:"<<endl;
    for (auto& i : nodes)
        cout<<"("<<i.first<<","<<i.second<<") ";
    cout<<endl;
    */
    N=nodes.size();
    costs=vector<vector<double>>(N, vector<double>(N, 0));
    Dijkstra dijk;
    for (int i=0; i<N; ++i)
        for (int j=0; j<N; ++j)
            if (i!=j)
                costs[i][j]=dijk.solve(nodes[i].first, nodes[j].first).cost();
    try {
        cplex.setParam(IloCplex::Threads, 1);
        cplex.setOut(env.getNullStream());
        cplex.setWarning(env.getNullStream());
        //cplex.setError(env.getNullStream());
        addVarsObjCoeffs();
        addConstraints();
    } catch (IloException& e) {
        cout<<"Concert exception caught: "<<e<<endl;
        exit(-1);
    } catch (...) {
        cout<<"Unknown exception caught"<<endl;
        exit(-1);
    }
}

void RouteReoptimizer::addConstraints() {
    // no loops
    for (int i=0; i<N; ++i) {
        IloRange noloop(env, 0, 0);
        noloop.setLinearCoef(var[i*N+i], 1);
        con.add(noloop);
    }
    // in-degree constraints
    for (int j=0; j<N; ++j) {
        IloNumVarArray vars(env);
        IloNumArray coeffs(env);
        for (int i=0; i<N; ++i) {
            vars.add(var[i*N+j]);
            coeffs.add(1);
        }
        IloRange indeg(env, 1, 1);
        indeg.setLinearCoefs(vars, coeffs);
        con.add(indeg);
        vars.end();
        coeffs.end();
    }
    // out-degree constraints
    for (int i=0; i<N; ++i) {
        IloNumVarArray vars(env);
        IloNumArray coeffs(env);
        for (int j=0; j<N; ++j) {
            vars.add(var[i*N+j]);
            coeffs.add(1);
        }
        IloRange outdeg(env, 1, 1);
        outdeg.setLinearCoefs(vars, coeffs);
        con.add(outdeg);
        vars.end();
        coeffs.end();
    }
    // force last node (depot) to connect to first node
    IloRange deptofirst(env, 1, 1);
    deptofirst.setLinearCoef(var[(N-1)*N], 1);
    con.add(deptofirst);
    // SECs
    cplex.use(TSPSolverSECLC(env, var, N));
    cplex.use(TSPSolverSECUC(env, var, N));
}

void RouteReoptimizer::addVarsObjCoeffs() {
    IloNumArray coeffs(env);
    for (int i=0; i<N; ++i) {
        for (int j=0; j<N; ++j) {
            var.add(IloNumVar(env, 0, 1, ILOINT));
            coeffs.add(costs[i][j]);
        }
    }
    obj.setLinearCoefs(var, coeffs);
    coeffs.end();
}

PlannedRoute RouteReoptimizer::buildReoptimizedRoute(const vector<int>& tour)
        const {
    vector<PlannedLink> reoptlinks;
    // everything the same up to the first node
    for (int i=0; i<=nodes[0].second; ++i)
        reoptlinks.push_back(route.links.at(i));    // reqs included!
    Dijkstra dijk;
    for (int i=1; i<tour.size()-1; ++i) {   // not include depot->first
        auto p=dijk.solve(nodes[tour[i-1]].first, nodes[tour[i]].first);
        for (const auto& l : p.links())
            reoptlinks.push_back({l, {}});
        reoptlinks.back().reqs=route.links[nodes[tour[i]].second].reqs;
    }
    PlannedRoute reopt(reoptlinks, route.start);
    if (reopt.finish()>route.finish()+1e-11) {
        cout<<setprecision(20);
        cout<<"reopt.finish(): "<<reopt.finish()<<endl;
        cout<<"route.finish(): "<<route.finish()<<endl;
    }
    //assert(reopt.finish()<=route.finish()+1e-11);
    assert(reopt.requests().size()==route.requests().size());
    return reopt;
}

PlannedRoute RouteReoptimizer::reoptimize() {
    try {
        model.add(obj);
        model.add(con);
        //cout<<"TSP solver: extracting model..."<<endl;
        cplex.extract(model);
        //cplex.exportModel("TSP.lp");
        if (cplex.solve()) {
            IloNumArray vals(env);
            cplex.getValues(vals, var);
            vector<vector<int>> xijs(N, vector<int>(N, 0));
            for (int k=0, i=0; i<N; ++i)
                for (int j=0; j<N; ++j, ++k)
                    if (i!=j && vals[k]>=1-1e-3)
                        xijs[i][j]=1;
            vals.end();
            return buildReoptimizedRoute(getTour(xijs));
        } else {
            cout<<"Failed to solve model"<<endl;
            exit(-1);
        }
    } catch (IloException& e) {
        cout<<"Concert exception caught: "<<e<<endl;
        exit(-1);
    } catch (...) {
        cout<<"Unknown exception caught"<<endl;
        exit(-1);
    }
    cout<<"RouteReoptimizer::solve(): control should never get here"<<endl;
    exit(-1);
}

vector<int> RouteReoptimizer::getTour(const vector<vector<int>>& xijs) {
    vector<int> tour={0};
    int curr=0;
    do {
        for (int j=0; j<xijs.size(); ++j) {
            if (xijs[curr][j]) {
                tour.push_back(j);
                curr=j;
                break;
            }
        }
    } while (curr!=0);
    assert(tour.back()==0);
    if (tour.size()!=xijs.size()+1) {
        cout<<"tour.size(): "<<tour.size()<<endl;
        cout<<"xijs.size()+1: "<<xijs.size()+1<<endl;
        cout<<"xijs:"<<endl;
        for (int i=0; i<xijs.size(); ++i)
            for (int j=0; j<xijs.size(); ++j)
                cout<<"xijs["<<i<<"]["<<j<<"]: "<<xijs[i][j]<<endl;
    }
    assert(tour.size()==xijs.size()+1);
    return tour;
}

