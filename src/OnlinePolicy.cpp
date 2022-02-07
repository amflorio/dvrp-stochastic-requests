#include <numeric>
#include "Dijkstra.h"
#include "OnlinePolicy.h"
#include "RouteReoptimizer.h"

using namespace std;

shared_ptr<Data> OnlinePolicy::data;
double OnlinePolicy::potential_=-1;

/* Returns marginal cost and updated links of PlannedRoute route after inserting
 * req into route by cheapest insertion. Does not check feasibility (concerning
 * delivery period) of the updated links. If not possible to insert req into
 * route because last link of route is being traversed at instant u, an empty
 * PlannedLink vector is returned to indicate this situation. */
/*
pair<double, vector<PlannedLink>> OnlinePolicy::cheapestInsertion(
        const PlannedRoute& route, const Request& req) const {
    assert(req.u<=route.finish());
    // route representation considering only the next node, all nodes where
    // requests have to be served, and the last node (depot)
    struct arc {
        int i, j;           // next node, nodes with request or last node
        int bidx, eidx;     // first and last PlannedLink of route to remove
                            // when inserting req between i and j
        double cost;
        arc(int i_, int j_, int bidx_, int eidx_, double cost_) : i{i_}, j{j_},
                bidx{bidx_}, eidx{eidx_}, cost{cost_} {}
    };
    vector<arc> arcs;
    int next=route.linkTraversed(req.u);
    if (next==route.links.size()-1)
        return {0, {}};
    int curr;
    do {
        curr=next+1;
        for (next=curr; route.links[next].reqs.size()==0
                &&next<route.links.size()-1; ++next);
        assert(next>=curr);
        int s=route.links.at(curr).l.s;
        int t=route.links.at(next).l.t;
        auto accdists=[](double dist, const PlannedLink& link)
                {return dist+link.l.d;};
        arcs.emplace_back(s, t, curr, next, accumulate(route.links.begin()+curr,
                route.links.begin()+next+1, 0.0, accdists));
    } while (next<route.links.size()-1);
    // compute additional cost when inserting req at any position
    Dijkstra dijk;
    for (auto& a : arcs)
        a.cost=dijk.cost(a.i,req.i)+dijk.cost(req.i,a.j)-a.cost;
    // find the place where insertion will be cheapest
    auto cheapest=min_element(arcs.begin(), arcs.end(),
            [](const arc& a1, const arc& a2){return a1.cost<a2.cost;});
    // build and return the new PlannedRoute considering the inserted request
    vector<PlannedLink> newlinks;
    for (int i=0; i<cheapest->bidx; ++i)
        newlinks.push_back(route.links[i]); // everything the same up to bidx
    Path p=dijk.solve(cheapest->i, req.i);
    for (const auto& l : p.links())         // add links to go to req.i
        newlinks.emplace_back(l, vector<Request>());
    newlinks.back().reqs.push_back(req);    // serve req
    p=dijk.solve(req.i, cheapest->j);
    for (const auto& l : p.links())         // add links to come back to j
        newlinks.emplace_back(l, vector<Request>());
    for (const auto& r : route.links[cheapest->eidx].reqs)
        newlinks.back().reqs.push_back(r);  // keep requests scheduled at j
    for (int i=cheapest->eidx+1; i<route.links.size(); ++i)
        newlinks.push_back(route.links[i]); // everything the same after eidx
    return {cheapest->cost, newlinks};
}
*/

pair<double, vector<PlannedLink>> OnlinePolicy::cheapestInsertion(
        const PlannedRoute& route, const Request& req) const {
    assert(req.u<=route.finish());
    // route representation considering only the next node, all nodes where
    // requests have to be served, and the last node (depot)
    struct arc {
        int i, j;           // next node, nodes with request or last node
        int bidx, eidx;     // first and last PlannedLink of route to remove
                            // when inserting req between i and j
        double cost;
        arc(int i_, int j_, int bidx_, int eidx_, double cost_) : i{i_}, j{j_},
                bidx{bidx_}, eidx{eidx_}, cost{cost_} {}
    };
    vector<arc> arcs;
    int next=route.linkTraversed(req.u);
    if (next==route.links.size()-1)
        return {0, {}};
    int curr;
    Dijkstra dijk;
    do {
        curr=next+1;
        for (next=curr; route.links[next].reqs.size()==0
                &&next<route.links.size()-1; ++next);
        assert(next>=curr);
        int s=route.links.at(curr).l.s;
        int t=route.links.at(next).l.t;
        arcs.emplace_back(s, t, curr, next, dijk.cost(s, t));
    } while (next<route.links.size()-1);
    // compute additional cost when inserting req at any position
    for (auto& a : arcs)
        a.cost=dijk.cost(a.i,req.i)+dijk.cost(req.i,a.j)-a.cost;
    // find the place where insertion will be cheapest
    auto cheapest=min_element(arcs.begin(), arcs.end(),
            [](const arc& a1, const arc& a2){return a1.cost<a2.cost;});
    // build and return the new PlannedRoute considering the inserted request
    Path p1=dijk.solve(cheapest->i, req.i);
    Path p2=dijk.solve(req.i, cheapest->j);
    vector<PlannedLink> newlinks;
    newlinks.reserve(cheapest->bidx+p1.links().size()+p2.links().size()
            +route.links.size()-(cheapest->eidx+1)+1);
    for (int i=0; i<cheapest->bidx; ++i)
        newlinks.push_back(route.links[i]); // everything the same up to bidx
    for (const auto& l : p1.links())        // add links to go to req.i
        newlinks.emplace_back(l, vector<Request>());
    newlinks.back().reqs.push_back(req);    // serve req
    for (const auto& l : p2.links())        // add links to come back to j
        newlinks.emplace_back(l, vector<Request>());
    for (const auto& r : route.links[cheapest->eidx].reqs)
        newlinks.back().reqs.push_back(r);  // keep requests scheduled at j
    for (int i=cheapest->eidx+1; i<route.links.size(); ++i)
        newlinks.push_back(route.links[i]); // everything the same after eidx
    return {cheapest->cost, newlinks};
}

void OnlinePolicy::reoptimize(Decision& dec, const State& s) const {
    assert(reopt);
    assert(dec.accept);
    assert(dec.routing.size()!=0);
    // only reoptimize if not a new route
    if (!s.vehics[dec.assign].links.empty()) {
        RouteReoptimizer reopter({dec.routing, s.vehics[dec.assign].start},
                s.req.u);
        dec.routing=reopter.reoptimize().links;
    }
}

