#ifndef DVRPLABEL_H
#define DVRPLABEL_H

#include <iostream>
#include <memory>
#include <vector>
#include "DVRPData.h"

class DVRPLabel {
    public:
        static std::shared_ptr<DVRPData> data;
        std::vector<int> visits;
        std::vector<int> nodes;
        double cost;
        double duals;
        std::vector<int> ngset;
        int ngbin=0;
        bool dominated=false;
        bool extended=false;
        DVRPLabel(std::vector<int>, double, double);
        bool dominates(const DVRPLabel& l) const;
        DVRPLabel extend(int, double, double) const;
        bool ngForbid(int j) const;
        void print() const;
        void updateCost(const double, const std::vector<double>&);
};

#endif

