#include <algorithm>
#include <ilcplex/ilocplex.h>
#include "Dijkstra.h"
#include "PlannedRoute.h"

using namespace std;

ILOSTLBEGIN

shared_ptr<Data> PlannedRoute::data;

double PlannedRoute::distanceLeft(double u) const {
    assert(u>=start);
    double t=start;
    double dleft=0;
    for (int i=0; i<links.size(); ++i) {
        t+=links[i].l.d/data->instance().speed();
        if (t>u) {
            if (dleft==0) {     // add proportionally
                double pleft=(t-u)/(links[i].l.d/data->instance().speed());
                if (pleft>1)
                    pleft=1;
                dleft+=pleft*links[i].l.d;
            } else
                dleft+=links[i].l.d;
        }
        for (const auto& r : links[i].reqs)
            t+=r.d;
    }
    return dleft;
}

/* Returns true iff route is feasible concerning delivery period. */
bool PlannedRoute::feasible() const {
    return finish()<=data->instance().period();
}

/* Returns arrival time at the depot. */
double PlannedRoute::finish() const {
    double t=start;
    for (const auto& l : links) {
        t+=l.l.d/data->instance().speed();
        for (const auto& r : l.reqs)
            t+=r.d;
    }
    return t;
}

/* Returns the index of the PlannedLink that is traversed at instant u. If at
 * instant u the vehicle is serving a request, the index of the last traversed
 * arc is returned. */
int PlannedRoute::linkTraversed(double u) const {
    assert(u>start);
    double t=start;
    for (int i=0; i<links.size(); ++i) {
        t+=links[i].l.d/data->instance().speed();
        for (const auto& r : links[i].reqs)
            t+=r.d;
        if (t>u)
            return i;
    }
    assert(false);
}

vector<int> PlannedRoute::linksTraversed(const vector<Request>& trj, double u)
        const {
    double dleft=distanceLeft(u);
    double scurr=data->instance().speed();
    double snew=dleft*scurr/(dleft+scurr*(data->instance().period()-finish()));
    double speed=scurr;
    vector<int> idxs;
    double t=start;
    int begin=0;
    for (int i=0; i<links.size(); ++i) {
        t+=links[i].l.d/speed;
        for (const auto& r : links[i].reqs)
            t+=r.d;
        while (t>trj.at(begin).u) {
            speed=snew;
            idxs.push_back(i);
            begin++;
            if (begin==trj.size())
                return idxs;
        }
    }
    return idxs;
}

double PlannedRoute::potential(const vector<vector<Request>>& trjs, double u)
        const {
    assert(trjs.size()!=0);
    double tot=0;
    for (const auto& trj : trjs)
        tot+=potential(trj, u);
    return tot/trjs.size();
}

double PlannedRoute::potential(const vector<Request>& trj, double u) const {
    if (trj.size()==0)
        return 0;
    assert(trj[0].u>=u);
    auto idxs=linksTraversed(trj, u);
    vector<double> potreqs;
    Dijkstra dijk;
    for (int i=0; i<trj.size()&&i<idxs.size(); ++i) {
        pair<double, double> nearest={-1, numeric_limits<double>::max()};
        for (int j=idxs.at(i); j<links.size(); j+=5) {
            double dist=dijk.cost(links[j].l.t, trj[i].i)
                    +dijk.cost(trj[i].i, links[j].l.t);
            if (dist<nearest.second)
                nearest={j, dist};
        }
        if (nearest.first==-1)
            break;
        for (int j=nearest.first-4; j<=nearest.first+4; j++)
            if (j>=0&&j<links.size()) {
                double dist=dijk.cost(links[j].l.t, trj[i].i)
                        +dijk.cost(trj[i].i, links[j].l.t);
                if (dist<nearest.second)
                    nearest={j, dist};
            }
        potreqs.push_back(trj.at(i).d+nearest.second/data->instance().speed());
    }
    sort(potreqs.begin(), potreqs.end());
    double budget=data->instance().period()-finish();
    double pot=0;
    for (auto p : potreqs)
        if (p<budget) {
            pot+=1;
            budget-=p;
        } else {
            pot+=budget/p;
            break;
        }
    assert(budget>=0);
    return pot;
}

double PlannedRoute::potential(const std::vector<std::vector<Request>>& trjs,
        const std::vector<PlannedRoute>& routes, double u) {
    assert(trjs.size()!=0);
    double tot=0;
    for (const auto& trj : trjs)
        tot+=potential(trj, routes, u);
    return tot/trjs.size();
}

double PlannedRoute::potential(const std::vector<Request>& trj,
        const std::vector<PlannedRoute>& routes, double u) {
    if (trj.empty())
        return 0;
    IloEnv env;
    IloModel model(env);
    IloNumVarArray var(env);
    IloRangeArray con(env);
    IloObjective obj(IloMaximize(env));
    for (const auto& r : routes) {  // cap constraint on each knapsack
        con.add(IloRange(env, -IloInfinity,
                data->instance().period()-r.finish()));
    }
    for (int i=0; i<trj.size(); ++i)    // selecting each item maximum once
        con.add(IloRange(env, -IloInfinity, 1));
    IloNumVarArray varsobj(env);
    IloNumArray valsobj(env);
    Dijkstra dijk;
    for (int k=0; k<routes.size(); ++k) {
        auto idxs=routes[k].linksTraversed(trj, u);
        IloNumVarArray vars(env);   // to setup cap constraints on routes[k]
        IloNumArray vals(env);
        for (int i=0; i<trj.size(); ++i) {
            pair<double, double> nearest={-1,
                    data->instance().period()*data->instance().speed()};
            if (i<idxs.size()) {
                for (int j=idxs.at(i); j<routes[k].links.size(); j+=5) {
                    double dist=dijk.cost(routes[k].links[j].l.t, trj[i].i)
                            +dijk.cost(trj[i].i, routes[k].links[j].l.t);
                    if (dist<nearest.second)
                        nearest={j, dist};
                }
                for (int j=nearest.first-4; j<=nearest.first+4; j++)
                    if (j>=0&&j<routes[k].links.size()) {
                        double dist=dijk.cost(routes[k].links[j].l.t, trj[i].i)
                                +dijk.cost(trj[i].i, routes[k].links[j].l.t);
                        if (dist<nearest.second)
                            nearest={j, dist};
                    }
            }
            IloNumVar v(env, 0, 1);
            varsobj.add(v);
            valsobj.add(1);
            con[routes.size()+i].setLinearCoef(v, 1);   // each item max once
            vars.add(v);
            vals.add(trj[i].d+nearest.second/data->instance().speed());
        }
        con[k].setLinearCoefs(vars, vals);
        vars.end();
        vals.end();
    }
    obj.setLinearCoefs(varsobj, valsobj);
    varsobj.end();
    valsobj.end();
    model.add(obj);
    model.add(con);
    IloCplex cplex(model);
    cplex.setParam(IloCplex::Threads, 1);
    cplex.setOut(env.getNullStream());
    //cplex.setParam(IloCplex::NumParam::TiLim, 120);
    //cplex.exportModel("lp.lp");
    if (!cplex.solve()) {
        cout<<"PlannedRoute::potential: can't solve multiple knapsack"<<endl;
        exit(-1);
    }
    auto res=cplex.getObjValue();
    env.end();
    return res;
}

/* Returns all requests (planned or already served). */
vector<Request> PlannedRoute::requests() const {
    vector<Request> reqs;
    for (const auto& l : links)
        for (const auto& r : l.reqs)
            reqs.push_back(r);
    return reqs;
}

ostream& operator<<(ostream& os, const PlannedRoute& r) {
    assert(r.links.size()>0);
    int nreq=0;
    for (const auto& l : r.links)
        nreq+=l.reqs.size();
    os<<"["<<nreq<<","<<r.start<<","<<r.links[0].l.s;
    for (const auto& l : r.links) {
        os<<"-"<<l.l.t;
        if (l.reqs.size()>0)
            os<<"("<<l.reqs.size()<<")";
    }
    os<<","<<r.finish()<<"]";
    return os;
}

