#ifndef SPBP_H
#define SPBP_H

#include <vector>
#include "BasePolicy.h"

class SPbP : public BasePolicy {
    private:
        int Hpot;
        std::vector<std::tuple<Decision, double>> decisionsPotentials(
                const State& s) const;
    public:
        SPbP(int hpot) : Hpot{hpot} {}
        Decision decide(const State& s) const override;
        std::vector<Decision> decisions(const State& s) const override;
        std::string texString() const override;
};

#endif

