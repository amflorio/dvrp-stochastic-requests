#ifndef INSTANCE_H
#define INSTANCE_H

#include <cassert>
#include <string>
#include <vector>
#include "Link.h"
#include "Request.h"

class Instance {
    private:
        friend class TikZExporter;
        const double PI=3.14159265359;
        const int UTI=0;    // spatially uniform, time-independent
        const int CTI=1;    // clustered, time-independent
        const int CTD=2;    // clustered, time-dependent
        const int NC=0;     // non-cluster
        const int C=1;      // clustered
        const int C1=2;     // cluster type 1
        const int C2=3;     // cluster type 2
        const int U=10*60;  // service period (in minutes)
        const double fDmean=10;    // mean duration of requests
        const double fDstd=2.5;    // std. deviation of duration of requests
        const double spd=20*1000/60;        // speed (meters/minute)
        std::string code_;  // instance code
        std::string network;
        int V;              // number of vertices in the road network graph
        int A;              // number of links in the road network graph
        double Lambda;      // overall rate (per minute) of dynamic requests
        int stDist;         // space-time distribution of dynamic requests
        int K;              // number of vehicles
        int dep;            // vertex that represents the depot
        double degDyn;      // degree of dynamism
        std::vector<double> xcoords;    // x,y coords (for plotting and ref nds)
        std::vector<double> ycoords;
        std::vector<double> lats;
        std::vector<double> lons;
        std::vector<std::tuple<double, double, int>> clusters;  //lat,lon,radius
        std::vector<int> type;          // type of each node: NC, C, C1 or C2
        std::vector<Link> links_;       // set of links
        std::vector<std::vector<Link>> linksFromV;  // links from a given vertex
        std::vector<std::vector<Link>> linksToV;    // links to a given vertex
        bool belongs(int i, const std::tuple<double, double, int>& cluster)
                const;
        double distanceLL(double lat1, double lon1, double lat2, double lon2)
                const;
        double distanceXY(double x1, double y1, double x2, double y2) const;
        int findNearestNodeLL(double lat, double lon) const;
        std::vector<Request> genRandomDynRequestsCTD(double u) const;
        std::vector<Request> genRandomDynRequestsCTI(double u) const;
        std::vector<Request> genRandomDynRequestsUTI(double u) const;
        std::pair<double, double> getXYCoords(double lat, double lon) const;
        void loadInstance();
        void loadRealWorld();
        void setupClusters();
        void setupDepot();
        int staticRequests() const {return (Lambda*U*(1-degDyn))/degDyn+0.5;}
        //int staticRequests() const {return Lambda*U*degDyn/(1-degDyn);}
        void usage() const;
    public:
        Instance(std::string code);
        std::string code() const {return code_;}
        int depot() const {return dep;}
        double distanceEuclidean(int i, int j) const
                {return distanceLL(lats[i], lons[i], lats[j], lons[j]);}
        int findNearestNodeXY(double x, double y) const;
        void generateRandomDynRequests() const;
        void generateRandomStaticRequests() const;
        const std::vector<Link>& links() const {return links_;}
        const std::vector<Link>& linksFrom(int i) const {
            assert(i>=0 && i<linksFromV.size());
            return linksFromV[i];
        }
        const std::vector<Link>& linksTo(int i) const {
            assert(i>=0 && i<linksToV.size());
            return linksToV[i];
        }
        double period() const {return U;}
        void printInfo() const;
        std::vector<Request> sampleDynRequests(double u) const;
        int sizeA() const {return A;}
        int sizeV() const {return V;}
        double speed() const {return spd;}
        //void testDistances() const;
        int vehicles() const {return K;}
        double xCoord(int i) const {return xcoords[i];}
        double yCoord(int i) const {return ycoords[i];}
};

#endif

