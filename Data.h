#ifndef DATA_H
#define DATA_H

#include "Instance.h"

class Data {
    private:
        Instance inst;
    public:
        Data(Instance i) : inst{std::move(i)} {}
        const Instance& instance() const {return inst;}
};

#endif

