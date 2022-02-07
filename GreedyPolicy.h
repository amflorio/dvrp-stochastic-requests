#ifndef GREEDYPOLICY_H
#define GREEDYPOLICY_H

#include <vector>
#include "BasePolicy.h"

class GreedyPolicy : public BasePolicy {
    private:
        std::vector<std::pair<Decision, double>> decisionsCosts(const State& s)
                const;
    public:
        Decision decide(const State& s) const override;
        std::vector<Decision> decisions(const State& s) const override;
        std::string texString() const override;
};

#endif

