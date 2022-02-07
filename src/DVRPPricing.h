#ifndef DVRPPRICING_H
#define DVRPPRICING_H

#include <memory>
#include <vector>
#include "DVRPData.h"
#include "DVRPLabel.h"
#include "DVRPLinearSolution.h"

struct DVRPPricingState {
    bool firstrun=true;
    std::vector<std::vector<DVRPLabel>> labels;
};

class DVRPPricing {
    private:
        const double my0=1e-6;
        static DVRPPricingState state;
        std::shared_ptr<DVRPData> data;
        DVRPLinearSolution sol;
        void addRootLabels(std::vector<std::vector<DVRPLabel>>&) const;
        std::vector<std::vector<DVRPLabel>>& initLabels() const;
        double knapsackBound(const DVRPLabel& l) const;
        void mydebug() const;
        static void removeDominated(vector<DVRPLabel>& labels);
        static void resetState() {state.firstrun=true; state.labels.clear();}
        static void setDominated(vector<DVRPLabel>& labels);
    public:
        DVRPPricing(std::shared_ptr<DVRPData> d, DVRPLinearSolution s)
                : data{d}, sol{std::move(s)} {}
        std::vector<DVRPRoute> solveExact() const;
        std::vector<DVRPRoute> solveHeuristic() const;
};

#endif

