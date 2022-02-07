#include <iostream>
#include "Dijkstra.h"
#include "DVRPSolver.h"
#include "MyopicPlanner.h"

using namespace std;

OfflinePlan MyopicPlanner::plan() {
    DVRPSolver dvrp(genGraph(), data->instance().vehicles(),
            //data->instance().period()*0.25, data->instance().period()*0.75);
            0, data->instance().period());
    auto sol=dvrp.solve();
    cout<<"DVRP solution:"<<endl;
    sol.print();
    rts=dvrp.allRoutes();
    cout<<"MyopicPlanner: "<<rts.size()<<" routes generated"<<endl;
    return {plannedRoutes(sol.routes)};
}

