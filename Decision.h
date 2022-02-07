#ifndef DECISION_H
#define DECISION_H

#include <vector>
#include "PlannedRoute.h"

class Decision {
    public:
        bool accept;
        int assign;
        std::vector<PlannedLink> routing;
        Decision(bool acc, int k, std::vector<PlannedLink> rting) : accept{acc},
                assign{k}, routing{move(rting)} {}
};

std::ostream& operator<<(std::ostream& os, const Decision& d);

#endif

