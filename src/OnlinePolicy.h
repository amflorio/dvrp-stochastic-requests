#ifndef ONLINEPOLICY_H
#define ONLINEPOLICY_H

#include <memory>
#include "Data.h"
#include "Decision.h"
#include "State.h"

class OnlinePolicy {
    protected:
        static std::shared_ptr<Data> data;
        bool reopt=false;
        static double potential_;
        std::pair<double, std::vector<PlannedLink>> cheapestInsertion(
                const PlannedRoute& route, const Request& req) const;
        void reoptimize(Decision& dec, const State& s) const;
    public:
        virtual Decision decide(const State& s) const=0;
        double potential() {return potential_;}
        static void setData(const std::shared_ptr<Data> d) {data=d;}
        void setReoptimize() {reopt=true;}
        virtual std::string texString() const=0;
        virtual ~OnlinePolicy() {}
};

#endif

