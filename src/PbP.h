#ifndef PBP_H
#define PBP_H

#include <vector>
#include "BasePolicy.h"

class PbP : public BasePolicy {
    private:
        int Hpot;
        std::vector<std::tuple<Decision, double>> decisionsPotentials(
                const State& s) const;
    public:
        PbP(int hpot) : Hpot{hpot} {}
        Decision decide(const State& s) const override;
        std::vector<Decision> decisions(const State& s) const override;
        std::string texString() const override;
};

#endif

