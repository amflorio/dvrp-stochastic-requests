#ifndef BASEPOLICY_H
#define BASEPOLICY_H

#include <vector>
#include "Decision.h"
#include "OnlinePolicy.h"
#include "State.h"

class BasePolicy : public OnlinePolicy {
    public:
        virtual std::vector<Decision> decisions(const State& s) const=0;
        virtual ~BasePolicy() {}
};

#endif

