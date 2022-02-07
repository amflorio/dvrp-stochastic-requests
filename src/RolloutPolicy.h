#ifndef ROLLOUTPOLICY_H
#define ROLLOUTPOLICY_H

#include "BasePolicy.h"
#include "OnlinePolicy.h"

class RolloutPolicy : public OnlinePolicy {
    private:
        const std::shared_ptr<BasePolicy> basepol;
        const int H;        // number of trajectories
        int rollout(const std::vector<PlannedRoute>& simRoutes, 
                const std::vector<std::vector<Request>>& simSamples) const;
    public:
        RolloutPolicy(std::shared_ptr<BasePolicy> p, int h) : basepol{p}, H{h}
                {}
        Decision decide(const State& s) const override;
        std::string texString() const override;
};

#endif

