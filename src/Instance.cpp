#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include "Instance.h"

using namespace std;

Instance::Instance(string code) : code_{code} {
    try {
        cout<<"Instance(): code: "<<code_<<endl;
        istringstream iss(code_);
        string part;
        getline(iss, part, '-');
        boost::to_lower(part);
        if (part.size()!=1||(part[0]!='e'&&part[0]!='v')) {
            cout<<"invalid network identifier"<<endl;
            exit(-1);
        }
        if (part[0]=='e') {
            network="eindhoven";
            cout<<"loading Eindhoven network ..."<<endl;
        } else if (part[0]=='v') {
            network="vienna";
            cout<<"loading Vienna network ..."<<endl;
        }
        loadRealWorld();
        part.erase();
        getline(iss, part, '-');
        Lambda=stof(part);
        cout<<"Lambda: "<<Lambda<<endl;
        part.erase();
        getline(iss, part, '-');
        boost::to_lower(part);
        if (part!="uti"&&part!="cti"&&part!="ctd") {
            cout<<"invalid space-time distribution"<<endl;
            exit(-1);
        }
        stDist=part=="uti"?UTI:part=="cti"?CTI:CTD;
        cout<<"space-time distribution: "<<part<<" ("<<stDist<<")"<<endl;
        part.erase();
        getline(iss, part, '-');
        K=stoi(part);
        if (K<2) {
            cout<<"invalid number of vehicles"<<endl;
            exit(-1);
        }
        cout<<"number of vehicles: "<<K<<endl;
        part.erase();
        getline(iss, part, '-');
        degDyn=stof(part)/100;
        if (degDyn<0||degDyn>1) {
            cout<<"invalid degree of dynamism"<<endl;
            exit(-1);
        }
        cout<<"degree of dynamism: "<<degDyn<<endl;
        cout<<"length of the service period: "<<U<<endl;
        setupClusters();
        setupDepot();
    } catch (...) {
        usage();
        exit(-1);
    }
}

bool Instance::belongs(int i, const tuple<double, double, int>& cluster) const {
    return distanceLL(lats[i], lons[i], get<0>(cluster), get<1>(cluster))
            <=get<2>(cluster);
}

double Instance::distanceLL(double lat1, double lon1, double lat2, double lon2)
        const {
    const double R=6371e3;          // Earth's radius in meters
    double x=(lon2-lon1)*cos((lat1+lat2)/2);
    double y=lat2-lat1;
    return sqrt(x*x+y*y)*R;
}

double Instance::distanceXY(double x1, double y1, double x2, double y2) const {
    double x=x2-x1;
    double y=y2-y1;
    return sqrt(x*x+y*y);
}

int Instance::findNearestNodeLL(double lat, double lon) const {
    pair<int, double> nearest {-1, numeric_limits<double>::infinity()};
    for (int i=0; i<V; ++i) {
        double dist=distanceLL(lat, lon, lats[i], lons[i]);
        if (dist<nearest.second) {
            nearest.first=i;
            nearest.second=dist;
        }
    }
    return nearest.first;
}

int Instance::findNearestNodeXY(double x, double y) const {
    pair<int, double> nearest {-1, numeric_limits<double>::infinity()};
    for (int i=0; i<V; ++i) {
        double dist=distanceXY(x, y, xcoords[i], ycoords[i]);
        if (dist<nearest.second) {
            nearest.first=i;
            nearest.second=dist;
        }
    }
    return nearest.first;
}

void Instance::generateRandomDynRequests() const {
    cout<<"generating random dynamic requests ..."<<endl;
    vector<Request> reqs=sampleDynRequests(0);
    for (const auto& req : reqs) 
        cout<<req.u<<" "<<req.i<<" "<<req.d<<endl;
    cout<<reqs.size()<<" requests generated (expected: "<<Lambda*U<<")"<<endl;
}

vector<Request> Instance::genRandomDynRequestsCTD(double u) const {
    // indexes of clustered (both types) and non-clustered nodes
    vector<int> clust1, clust2, nonclust;
    for (int i=0; i<V; ++i)
        if (type[i]==C1)
            clust1.push_back(i);
        else if (type[i]==C2)
            clust2.push_back(i);
        else
            nonclust.push_back(i);
    mt19937 gen((random_device()()));
    /*
    static long long seed=0;
    mt19937 gen(seed++);
    */
    uniform_real_distribution<double> unifDist;
    uniform_int_distribution<int> nodeC1Dist(0, clust1.size()-1);
    uniform_int_distribution<int> nodeC2Dist(0, clust2.size()-1);
    uniform_int_distribution<int> nodeNCDist(0, nonclust.size()-1);
    normal_distribution<double> durDist(fDmean, fDstd);
    vector<Request> reqs;
    while (true) {
        double unif=unifDist(gen);
        u+=-(1/Lambda)*log(unif);
        if (u>U)
            break;
        unif=unifDist(gen);
        int i;
        if (unif>0.5) {     // request from a cluster
            unif=unifDist(gen);
            if (unif>((0.8-0.2)/600)*u+0.2) {       // C1 (early) cluster
                i=clust1[nodeC1Dist(gen)];
                while (i==dep)
                    i=clust1[nodeC1Dist(gen)];
            } else {                                // C2 (late) cluster
                i=clust2[nodeC2Dist(gen)];
                while (i==dep)
                    i=clust2[nodeC2Dist(gen)];
            }
        } else {            // request from a non-cluster
            i=nonclust[nodeNCDist(gen)];
            while (i==dep)
                i=nonclust[nodeNCDist(gen)];
        }
        double d=durDist(gen);
        while (d<=0)
            d=durDist(gen);
        reqs.emplace_back(u, i, d);
    }
    return reqs;
}

vector<Request> Instance::genRandomDynRequestsCTI(double u) const {
    // indexes of clustered and non-clustered nodes
    vector<int> clust, nonclust;
    for (int i=0; i<V; ++i)
        if (type[i]==C)
            clust.push_back(i);
        else
            nonclust.push_back(i);
    mt19937 gen((random_device()()));
    /*
    static long long seed=0;
    mt19937 gen(seed++);
    */
    uniform_real_distribution<double> unifDist;
    uniform_int_distribution<int> nodeCDist(0, clust.size()-1);
    uniform_int_distribution<int> nodeNCDist(0, nonclust.size()-1);
    normal_distribution<double> durDist(fDmean, fDstd);
    vector<Request> reqs;
    while (true) {
        double unif=unifDist(gen);
        u+=-(1/Lambda)*log(unif);
        if (u>U)
            break;
        unif=unifDist(gen);
        int i;
        if (unif>0.5) {     // request from a cluster
            i=clust[nodeCDist(gen)];
            while (i==dep)
                i=clust[nodeCDist(gen)];
        } else {            // request from a non-cluster
            i=nonclust[nodeNCDist(gen)];
            while (i==dep)
                i=nonclust[nodeNCDist(gen)];
        }
        double d=durDist(gen);
        while (d<=0)
            d=durDist(gen);
        reqs.emplace_back(u, i, d);
    }
    return reqs;
}

vector<Request> Instance::genRandomDynRequestsUTI(double u) const {
    mt19937 gen((random_device()()));
    /*
    static long long seed=7;
    mt19937 gen(seed++);
    */
    uniform_real_distribution<double> unifDist;
    uniform_int_distribution<int> nodeDist(0, V-1);
    normal_distribution<double> durDist(fDmean, fDstd);
    vector<Request> reqs;
    while (true) {
        double unif=unifDist(gen);
        u+=-(1/Lambda)*log(unif);
        if (u>U)
            break;
        int i=nodeDist(gen);
        while (i==dep)
            i=nodeDist(gen);
        double d=durDist(gen);
        while (d<=0)
            d=durDist(gen);
        reqs.emplace_back(u, i, d);
    }
    return reqs;
}

void Instance::generateRandomStaticRequests() const {
    cout<<"generating "<<staticRequests()<<" random static requests ..."<<endl;
    mt19937 gen((random_device()()));
    //mt19937 gen(std::hash<string>()(code_));
    uniform_int_distribution<int> nodeDist(0, V-1);
    normal_distribution<double> durDist(fDmean, fDstd);
    for (int j=0; j<staticRequests(); ++j) {
        int i=nodeDist(gen);
        while (i==dep)
            i=nodeDist(gen);
        double d=durDist(gen);
        while (d<=0)
            d=durDist(gen);
        cout<<"0 "<<i<<" "<<d<<endl;
    }
}

pair<double, double> Instance::getXYCoords(double lat, double lon) const {
    const double lat0=network=="vienna"?(48.2206/180)*PI:(51.438/180)*PI;
    const double offsetx=network=="vienna"?-0.185:-0.059;
    const double offsety=network=="vienna"?-0.841:-0.897;
    const double scaling=network=="vienna"?25000:25000;
    return {(lon*cos(lat0)+offsetx)*scaling, (lat+offsety)*scaling};
}

void Instance::loadRealWorld() {
    string filename="../in/coords/"+network+".xy";
    ifstream fcoords(filename);
    if (!fcoords.is_open()) {
        cout<<"loadRealWorld(): unable to open file "<<filename<<endl;
        exit(-1);
    }
    fcoords>>V;
    xcoords=vector<double>(V, 0);
    ycoords=vector<double>(V, 0);
    lats=vector<double>(V, 0);
    lons=vector<double>(V, 0);
    int n;
    for (int i=0; i<V; ++i) {
        fcoords>>n>>lats[i]>>lons[i];
        auto xy=getXYCoords(lats[i], lons[i]);
        xcoords[i]=xy.first;
        ycoords[i]=xy.second;
        if (n!=i) {
            cout<<"loadRealWorld(): inconsistency: n,i="<<n<<","<<i<<endl;
            exit(-1);
        }
    }
    filename="../in/dists/"+network+".d";
    ifstream fdists(filename);
    if (!fdists.is_open()) {
        cout<<"loadRealWorld(): unable to open file "<<filename<<endl;
        exit(-1);
    }
    linksFromV=vector<vector<Link>>(V, vector<Link>());
    linksToV=vector<vector<Link>>(V, vector<Link>());
    fdists>>A;
    for (int l=0; l<A; ++l) {
        int s, t;
        double d;
        fdists>>s>>t>>d;
        Link link(l, s, t, d);
        links_.push_back(link);
        linksFromV.at(link.s).push_back(link);
        linksToV.at(link.t).push_back(link);
    }
    // consistency check: every vertex need to have at least 1 link to/from
    for (int i=0; i<V; ++i)
        if (linksFromV[i].size()==0 || linksToV[i].size()==0) {
            cout<<"loadRealWorld(): inconsistency"<<endl;
            exit(-1);
        }
}

vector<Request> Instance::sampleDynRequests(double u) const {
    return stDist==UTI?genRandomDynRequestsUTI(u)
            :stDist==CTI?genRandomDynRequestsCTI(u)
            :genRandomDynRequestsCTD(u);
}

void Instance::setupClusters() {
    cout<<"setting up node types based on spatial dist. and clusters ..."<<endl;
    if (stDist!=UTI) {
        if (network=="eindhoven")
            clusters={{51.436879, 5.472860, 900},   // Center
                {51.445041, 5.454482, 600},         // Strijp-S
                {51.467228, 5.473936, 1000},        // Woensel-Zuid
                {51.423800, 5.492604, 750}};        // Heistraat
        else if (network=="vienna")
            clusters={{48.249999, 16.443487, 3000}, // Kagraner Platz (Early)
                {48.192414, 16.315842, 3000}};      // Westbahnhof (Late)
        /*
            clusters={{48.208153, 16.373294, 1400}, // Stephansplatz
                {48.192414, 16.315842, 1400},       // Westbahnhof
                {48.279066, 16.410855, 2100},       // Floridsdorf
                {48.176011, 16.377125, 1400},       // Favoriten
                {48.230560, 16.328330, 1000},       // Gersthof
                {48.165604, 16.315499, 1200}};      // Hetzendorf
        */
        // converting lat,lon to radians
        for (auto& cluster : clusters) {
            get<0>(cluster)=(get<0>(cluster)/180)*PI;
            get<1>(cluster)=(get<1>(cluster)/180)*PI;
        }
    }
    // determine node types based on spatial distribution and clusters
    type=vector<int>(V, 0);
    for (int i=0; i<V; ++i) {
        if (stDist==UTI)
            type[i]=NC;
        else if (stDist==CTI) {
            type[i]=NC;
            for (const auto& cluster : clusters)
                if (belongs(i, cluster)) {
                    type[i]=C;
                    break;
                }
        } else {        // CTD
            type[i]=NC;
            for (int j=0; j<clusters.size(); ++j)
                if (belongs(i, clusters[j])) {
                    if (j%2==0)
                        type[i]=C1;
                    else
                        type[i]=C2;
                    break;
                }
        }
    }
}

void Instance::setupDepot() {
    pair<double, double> ll=network=="eindhoven"
            ?pair<double,double>({51.434555,5.510250})
            :pair<double,double>({48.178808,16.438460});
    dep=findNearestNodeLL((ll.first/180)*PI, (ll.second/180)*PI);
}

void Instance::usage() const {
    cout<<"invalid instance code `"<<code_<<"'"<<endl;
    cout<<"code should be in the format `n-rate-dist-K-dd', where:"<<endl;
    cout<<"n\t network code (eg `e' for Eindhoven and `v' for Vienna)"<<endl;
    cout<<"rate\t rate of requests, per minute (eg 1.5)"<<endl;
    cout<<"dist\t space-time distribution of requests (`uti', `cti' or `ctd')"
            <<endl;
    cout<<"K\t number of vehicles"<<endl;
    cout<<"dd\t degree of dynamism"<<endl;
}

/*
void Instance::testDistances() const {
    pair<double, double> llorig {48.228249, 16.326659};     // Czartoryskig.6
    pair<double, double> lldest {48.269926, 16.426735};     // Giefingg.7
    int orig=findNearestNode((llorig.first/180)*PI, (llorig.second/180)*PI);
    int dest=findNearestNode((lldest.first/180)*PI, (lldest.second/180)*PI);
    Dijkstra dijk;
    cout<<"shortest-path cost: "<<dijk.solve(orig, dest).cost()<<endl;
    cin.get();
}
*/

