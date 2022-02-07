#ifndef DVRPLINEARSOLUTION_H
#define DVRPLINEARSOLUTION_H

#include <vector>
#include "DVRPRoute.h"

struct DVRPLinearSolution {
    double val;
    std::vector<std::pair<double, DVRPRoute>> routes;
    double kDual;
    std::vector<double> pttDuals;
    DVRPLinearSolution(double v, std::vector<std::pair<double, DVRPRoute>> rts,
            double kD, std::vector<double> pttDls) : val{v},
            routes{std::move(rts)}, kDual{kD}, pttDuals{std::move(pttDls)} {}
};

#endif

