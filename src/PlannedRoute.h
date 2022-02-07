#ifndef PLANNEDROUTE_H
#define PLANNEDROUTE_H

#include <iostream>
#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include "Data.h"
#include "Link.h"
#include "Request.h"

class PlannedLink {
    friend class boost::serialization::access;
    private:
        template<class Archive> void serialize(Archive& ar,
                const unsigned int version) {
            ar & l;
            ar & reqs;
        }
        PlannedLink() {}
    public:
        Link l;
        std::vector<Request> reqs;  // to serve when arriving at link endpoint
        PlannedLink(Link lnk, std::vector<Request> rqs) : l{std::move(lnk)},
                reqs{std::move(rqs)} {}
};

class PlannedRoute {
    friend class boost::serialization::access;
    private:
        static std::shared_ptr<Data> data;
        template<class Archive> void serialize(Archive& ar, 
                const unsigned int version) {
            ar & links;
            ar & start;
        }
        PlannedRoute() {}
        double distanceLeft(double u) const;
        std::vector<int> linksTraversed(const std::vector<Request>& trj,
                double u) const;
        std::vector<std::vector<int>> nodesOfInterest(
                const std::vector<Request>& trj, double u) const;
        double potential(const std::vector<Request>& trj, double u) const;
        static double potential(const std::vector<Request>& trj,
                const std::vector<PlannedRoute>& routes, double u);
    public:
        std::vector<PlannedLink> links;
        double start;
        PlannedRoute(std::vector<PlannedLink> lnks, double s)
                : links{std::move(lnks)}, start{s} {}
        bool feasible() const;
        double finish() const;
        int linkTraversed(double u) const;
        double potential(const std::vector<std::vector<Request>>& trjs,
                double u) const;
        static double potential(const std::vector<std::vector<Request>>& trjs,
                const std::vector<PlannedRoute>& routes, double u);
        std::vector<Request> requests() const;
        static void setData(const std::shared_ptr<Data> d) {data=d;}
};

std::ostream& operator<<(std::ostream& os, const PlannedRoute& r);

#endif

