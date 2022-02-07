#ifndef PFAPOLICY_H
#define PFAPOLICY_H

#include <vector>
#include "BasePolicy.h"
#include "OfflinePlan.h"

class PFAPolicy : public BasePolicy {
    private:
        const OfflinePlan plan;
        const int H;
        double gamma=0;             // PFA coefficient
        Decision decide(const State& s, double g) const;
        std::vector<std::pair<Decision, double>> decisionsPotentials(
                const State& s, double g) const;
        int simulateReward(const std::vector<PlannedRoute>& routes,
                const std::vector<std::vector<Request>>& trj, double g);
    public:
        PFAPolicy(OfflinePlan pln, int h);
        Decision decide(const State& s) const override;
        std::vector<Decision> decisions(const State& s) const override;
        std::string texString() const override;
};

#endif

