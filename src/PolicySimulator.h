#ifndef POLICYSIMULATOR_H
#define POLICYSIMULATOR_H

#include <memory>
#include "Data.h"
#include "OfflinePlan.h"
#include "OnlinePolicy.h"
#include "Request.h"
#include "SimResults.h"
#include "State.h"

class PolicySimulator {
    friend class SmartPolicy;
    private:
        static std::shared_ptr<Data> data;
        std::shared_ptr<OnlinePolicy> policy;
        void setIdleSaveRequests(std::vector<PlannedRoute>& vehics, double u,
                std::vector<Request>& served) const;
    public:
        PolicySimulator(std::shared_ptr<OnlinePolicy> p) : policy{p} {}
        static void setData(const std::shared_ptr<Data> d) {data=d;}
        SimResults simulate(const OfflinePlan& plan,
                const std::vector<Request>& dynreqs) const;
        static void validateDecision(const Decision& dec,
                const std::vector<PlannedRoute>& vehics, const Request& r);
};

#endif

