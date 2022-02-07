#ifndef STATE_H
#define STATE_H

#include <vector>
#include "PlannedRoute.h"
#include "Request.h"

class State {
    public:
        std::vector<PlannedRoute> vehics;
        Request req;
        State(std::vector<PlannedRoute> vhcs, Request r)
                : vehics{std::move(vhcs)}, req{std::move(r)} {}
};

#endif

