#ifndef MYOPICPLANNER_H
#define MYOPICPLANNER_H

#include <vector>
#include "OfflinePlan.h"
#include "OfflinePlanner.h"
#include "Request.h"

class MyopicPlanner : public OfflinePlanner {
    private:
        std::vector<DVRPRoute> rts;
    public:
        MyopicPlanner(std::vector<Request> rqs) : OfflinePlanner{std::move(rqs)}
                {}
        OfflinePlan plan() override;
        std::vector<DVRPRoute> routes() const {return rts;}
};

#endif

