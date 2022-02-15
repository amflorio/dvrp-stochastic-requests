#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include "Data.h"
#include "Dijkstra.h"
#include "GreedyPolicy.h"
#include "Instance.h"
#include "MPotentialPlanner.h"
#include "MyopicPlanner.h"
#include "Path.h"
#include "PbP.h"
#include "PFAPolicy.h"
#include "PolicySimulator.h"
#include "PotentialPlanner.h"
#include "Request.h"
#include "RolloutPolicy.h"
#include "SimResults.h"
#include "SPbP.h"
#include "Stopwatch.h"
#include "TikZExporter.h"

using namespace std;

void preprocessSPs(shared_ptr<Data> data);
pair<vector<Request>, vector<Request>> readRequests(string filename);
void shareData(shared_ptr<Data> data);

int main(int argc, char* argv[]) {
    if (argc<2 || stoi(argv[1])<0 || stoi(argv[1])>33) {
        cout<<"usage: "<<argv[0]<<" <mode> [...]"<<endl;
        cout<<"where <mode> ="<<endl;
        cout<<"\t 0   Export Network Instance to TeX/TikZ"<<endl;
        cout<<"\t 1   Generate Random Static Requests"<<endl;
        cout<<"\t 2   Generate Random Dynamic Requests"<<endl;
        cout<<"\t 3   Create Myopic Offline Plan"<<endl;
        cout<<"\t 4   Create (simple) Potential-based Offline Plan"<<endl;
        cout<<"\t 5   Simulate: Greedy Policy (GP)"<<endl;
        cout<<"\t 6   Simulate: GP, with Reopt"<<endl;
        cout<<"\t 7   Simulate: Rollout on GP (H=10)"<<endl;
        cout<<"\t 8   Simulate: Rollout on GP (H=25)"<<endl;
        cout<<"\t 9   Simulate: Rollout on GP (H=50)"<<endl;
        cout<<"\t10   Simulate: Rollout on GP (H=100)"<<endl;
        cout<<"\t11   Simulate: Rollout on GP, with Reopt (H=10)"<<endl;
        cout<<"\t12   Simulate: Rollout on GP, with Reopt (H=25)"<<endl;
        cout<<"\t13   Simulate: Rollout on GP, with Reopt (H=50)"<<endl;
        cout<<"\t14   Simulate: Rollout on GP, with Reopt (H=100)"<<endl;
        cout<<"\t18   Create (multiple) Potential-based Offline Plan"<<endl;
        cout<<"\t19   Simulate: Simplified Potential-based Policy (S-PbP, H=50)"
                <<endl;
        cout<<"\t20   Simulate: S-PbP (H=100)"<<endl;
        cout<<"\t21   Simulate: PbP (H=50)"<<endl;
        cout<<"\t22   Simulate: PbP (H=100)"<<endl;
        cout<<"\t23   Export Offline Plan to TeX/TikZ"<<endl;
        /* New policies for the revised version of the paper. */
        cout<<"\t24   Simulate: PFA (CI)"<<endl;
        cout<<"\t25   Simulate: PFA (Reopt)"<<endl;
        cout<<"\t26   Simulate: Rollout on PFA (H=10)"<<endl;
        cout<<"\t27   Simulate: Rollout on PFA (H=25)"<<endl;
        cout<<"\t28   Simulate: Rollout on PFA (H=50)"<<endl;
        cout<<"\t29   Simulate: Rollout on PFA (H=100)"<<endl;
        cout<<"\t30   Simulate: Rollout on PFA, with Reopt (H=10)"<<endl;
        cout<<"\t31   Simulate: Rollout on PFA, with Reopt (H=25)"<<endl;
        cout<<"\t32   Simulate: Rollout on PFA, with Reopt (H=50)"<<endl;
        cout<<"\t33   Simulate: Rollout on PFA, with Reopt (H=100)"<<endl;
        return 0;
    }
    int mode=stoi(argv[1]);
    if (mode==0 && argc!=3) {
        cout<<"[mode=0]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code>"<<endl;
        return 0;
    }
    if (mode==1 && argc!=3) {
        cout<<"[mode=1]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code>"<<endl;
        return 0;
    }
    if (mode==2 && argc!=3) {
        cout<<"[mode=2]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code>"<<endl;
        return 0;
    }
    if ((mode==3||mode==4||mode==18) && argc!=6) {
        cout<<"[mode=3..4,18]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code> <reqs. file>"
                " <routes file> <offl. plan file>"<<endl;
        return 0;
    }
    if (((mode>=5&&mode<=14)||(mode>=19&&mode<=22)||(mode>=24&&mode<=33))
            && argc!=5) {
        cout<<"[mode=5...14,19...22,24...33]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code> <reqs. file>"
                " <offl. plan file>"<<endl;
        return 0;
    }
    if (mode==23&&argc!=4) {
        cout<<"[mode=23]"<<endl;
        cout<<"usage: "<<argv[0]<<" <mode> <inst. code> <offl. plan file>"
                <<endl;
        return 0;
    }
    streambuf *old=cout.rdbuf();
    stringstream ss;
    if (mode==0||mode==23)
        cout.rdbuf(ss.rdbuf());
    shared_ptr<Data> data=make_shared<Data>(Instance(argv[2]));
    //shared_ptr<Data> data {new Data(Instance(argv[2]))};
    shareData(data);
    cout.rdbuf(old);
    if (mode==0) {
        TikZExporter exp(data->instance());
        exp.exportNetwork();
    } else if (mode==1) {
        data->instance().generateRandomStaticRequests();
    } else if (mode==2) {
        data->instance().generateRandomDynRequests();
    } else if (mode==3) {
        Stopwatch sw;
        auto reqs=readRequests(argv[3]);
        MyopicPlanner planner(reqs.first);
        auto plan=planner.plan();
        cout<<"offline plan generated! saving to `"<<argv[5]<<"' ..."<<endl;
        {
            ofstream ofs(argv[5]);
            assert(ofs.good());
            boost::archive::text_oarchive oa(ofs);
            oa<<plan;
        }
        cout<<"offline plan saved!"<<endl;
        auto routes=planner.routes();
        cout<<"saving pool of routes to `"<<argv[4]<<"' ..."<<endl;
        {
            ofstream ofs(argv[4]);
            assert(ofs.good());
            boost::archive::text_oarchive oa(ofs);
            oa<<routes;
        }
        cout<<"pool of routes saved!"<<endl;
        sw.stop();
        cout<<"elapsed time: "<<sw.elapsedSeconds()<<" s"<<endl;
    } else if (mode==4||mode==18) {
        Stopwatch sw;
        auto reqs=readRequests(argv[3]);
        cout<<"reading routes from "<<argv[4]<<" ..."<<endl;
        vector<DVRPRoute> routes;
        {
            ifstream ifs(argv[4]);
            assert(ifs.good());
            boost::archive::text_iarchive ia(ifs);
            ia>>routes;
        }
        cout<<routes.size()<<" routes read!"<<endl;
        shared_ptr<OfflinePlanner> planner;
        if (mode==4)
            planner=make_shared<PotentialPlanner>(reqs.first, routes);
        else if (mode==18)
            planner=make_shared<MPotentialPlanner>(reqs.first, routes);
        auto plan=planner->plan();
        cout<<"offline plan generated! saving to `"<<argv[5]<<"' ..."<<endl;
        {
            ofstream ofs(argv[5]);
            assert(ofs.good());
            boost::archive::text_oarchive oa(ofs);
            oa<<plan;
        }
        cout<<"offline plan saved!"<<endl;
        sw.stop();
        cout<<"elapsed time: "<<sw.elapsedSeconds()<<" s"<<endl;
    } else if((mode>=5&&mode<=14)||(mode>=19&&mode<=22)||(mode>=24&&mode<=33)) {
        preprocessSPs(data);
        auto reqs=readRequests(argv[3]);
        OfflinePlan plan({});
        cout<<"loading offline plan from "<<argv[4]<<" ..."<<endl;
        ifstream ifs(argv[4]);
        assert(ifs.good());
        {
            boost::archive::text_iarchive ia(ifs);
            ia>>plan;
        }
        cout<<"offline plan loaded!"<<endl;
        plan.print();
        shared_ptr<OnlinePolicy> pol;
        if (mode==5) {
            pol=make_shared<GreedyPolicy>();
        } else if (mode==6) {
            pol=make_shared<GreedyPolicy>();
            pol->setReoptimize();
        } else if (mode==7) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 10);
        } else if (mode==8) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 25);
        } else if (mode==9) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 50);
        } else if (mode==10) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 100);
        } else if (mode==11) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 10);
            pol->setReoptimize();
        } else if (mode==12) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 25);
            pol->setReoptimize();
        } else if (mode==13) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 50);
            pol->setReoptimize();
        } else if (mode==14) {
            pol=make_shared<RolloutPolicy>(make_shared<GreedyPolicy>(), 100);
            pol->setReoptimize();
        } else if (mode==19) {
            pol=make_shared<SPbP>(50);
            pol->setReoptimize();
        } else if (mode==20) {
            pol=make_shared<SPbP>(100);
            pol->setReoptimize();
        } else if (mode==21) {
            pol=make_shared<PbP>(50);
            pol->setReoptimize();
        } else if (mode==22) {
            pol=make_shared<PbP>(100);
            pol->setReoptimize();
        } else if (mode==24) {
            pol=make_shared<PFAPolicy>(plan, 100);
        } else if (mode==25) {
            pol=make_shared<PFAPolicy>(plan, 100);
            pol->setReoptimize();
        // Rollout on PFA: first number PFA tuning H, second number rollout H
        } else if (mode==26) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    10);
        } else if (mode==27) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    25);
        } else if (mode==28) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    50);
        } else if (mode==29) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    100);
        } else if (mode==30) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    10);
            pol->setReoptimize();
        } else if (mode==31) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    25);
            pol->setReoptimize();
        } else if (mode==32) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    50);
            pol->setReoptimize();
        } else if (mode==33) {
            pol=make_shared<RolloutPolicy>(make_shared<PFAPolicy>(plan, 100),
                    100);
            pol->setReoptimize();
        } else {
            cout<<"invalid mode"<<endl;
            return -1;
        }
        PolicySimulator sim(pol);
        auto res=sim.simulate(plan, reqs.second);
        res.print();
    } else if (mode==23) {
        OfflinePlan plan({});
        ifstream ifs(argv[3]);
        assert(ifs.good());
        {
            boost::archive::text_iarchive ia(ifs);
            ia>>plan;
        }
        TikZExporter exp(data->instance());
        exp.exportOfflinePlan(plan);
    } else {
        cout<<"invalid mode"<<endl;
        return -1;
    }
    return 0;
}

void preprocessSPs(shared_ptr<Data> data) {
    cout<<"pre-processing shortest-paths ..."<<endl;
    Dijkstra dijk;
    for (int i=0; i<data->instance().sizeV(); ++i) {
        if (i&&i%2000==0)
            cout<<i<<"/"<<data->instance().sizeV()<<" done"<<endl;
        dijk.solve(i, 0);
    }
    cout<<"finished pre-processing shortest-paths"<<endl;
}

pair<vector<Request>, vector<Request>> readRequests(string filename) {
    ifstream ifs(filename);
    assert(ifs.good());
    vector<Request> sreqs, dreqs;
    double u, d;
    int i;
    while (ifs>>u>>i>>d)
        if (u==0)
            sreqs.emplace_back(u, i, d);
        else
            dreqs.emplace_back(u, i, d);
    cout<<"readRequests: read "<<sreqs.size()<<" static and "<<dreqs.size()
            <<" dynamic requests"<<endl;
    return {sreqs, dreqs};
}

void shareData(shared_ptr<Data> data) {
    Dijkstra::setData(data);
    OfflinePlan::setData(data);
    OfflinePlanner::setData(data);
    OnlinePolicy::setData(data);
    Path::setData(data);
    PlannedRoute::setData(data);
    PolicySimulator::setData(data);
    SimResults::setData(data);
}

