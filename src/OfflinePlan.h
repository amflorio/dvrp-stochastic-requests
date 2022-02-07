#ifndef OFFLINEPLAN_H
#define OFFLINEPLAN_H

#include <vector>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/vector.hpp>
#include "Data.h"
#include "PlannedRoute.h"
#include "Request.h"

class OfflinePlan {
    friend class boost::serialization::access;
    private:
        static std::shared_ptr<Data> data;
        template<class Archive> void serialize(Archive& ar,
                const unsigned int version) {
            ar & routes;
        }
    public:
        std::vector<PlannedRoute> routes;
        OfflinePlan(std::vector<PlannedRoute> rts) : routes{move(rts)} {}
        void print() const;
        std::vector<Request> requests() const;
        static void setData(const std::shared_ptr<Data> d) {data=d;}
};

#endif

