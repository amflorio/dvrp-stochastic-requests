#ifndef OFFLINEPLANNER_H
#define OFFLINEPLANNER_H

#include <memory>
#include <vector>
#include "Data.h"
#include "DVRPRoute.h"
#include "OfflinePlan.h"
#include "Request.h"

class OfflinePlanner {
    protected:
        static std::shared_ptr<Data> data;
        const std::vector<Request> reqs;
        std::vector<std::vector<double>> genGraph() const;
        PlannedRoute plannedRoute(const DVRPRoute& r) const;
        std::vector<PlannedRoute> plannedRoutes(
                const std::vector<DVRPRoute>& routes) const;
    public:
        OfflinePlanner(std::vector<Request> rqs) : reqs{std::move(rqs)} {}
        virtual OfflinePlan plan()=0;
        static void setData(const std::shared_ptr<Data> d) {data=d;}
        virtual ~OfflinePlanner() {}
};

#endif

