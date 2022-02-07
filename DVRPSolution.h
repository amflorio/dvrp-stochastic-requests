#ifndef DVRPSOLUTION_H
#define DVRPSOLUTION_H

#include <vector>
#include "DVRPRoute.h"

class DVRPSolution {
    public:
        double val;
        std::vector<DVRPRoute> routes;
        void print() const;
};

#endif

