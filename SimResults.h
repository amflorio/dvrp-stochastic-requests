#ifndef SIMRESULTS_H
#define SIMRESULTS_H

#include <chrono>
#include <memory>
#include <vector>
#include "Data.h"

using std::move;
using std::vector;
using hrclock=std::chrono::high_resolution_clock;

class SimResults {
    private:
        static std::shared_ptr<Data> data;
        vector<Request> accepted, rejected, static_;
        vector<hrclock::duration> dectimes;
        std::vector<std::pair<int, int>> cumulativeAcceptance() const;
    public:
        SimResults(vector<Request> acc, vector<Request> rej,
                vector<Request> stat, vector<hrclock::duration> dtimes) :
                accepted{move(acc)}, rejected{move(rej)}, static_{move(stat)},
                dectimes{move(dtimes)} {}
        static double avgDecisionTime(const std::vector<hrclock::duration>&);
        static double maxDecisionTime(const std::vector<hrclock::duration>&);
        void print() const;
        static void setData(const std::shared_ptr<Data> d) {data=d;}
};

#endif

