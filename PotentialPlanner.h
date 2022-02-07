#ifndef POTENTIALPLANNER_H
#define POTENTIALPLANNER_H

#include <vector>
#include "OfflinePlan.h"
#include "OfflinePlanner.h"
#include "Request.h"

class PotentialPlanner : public OfflinePlanner {
    private:
        std::vector<DVRPRoute> routes;
        std::vector<DVRPRoute> solveMaxPotential(int N) const;
        double solveMinDuration(int N) const;
    public:
        PotentialPlanner(std::vector<Request> rqs, std::vector<DVRPRoute> rts);
        OfflinePlan plan() override;
};

#endif

