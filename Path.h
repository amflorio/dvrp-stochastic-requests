#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <memory>
#include <vector>
#include "Data.h"
#include "Link.h"

class Path {
    friend std::ostream& operator<<(std::ostream& os, const Path& p);
    private:
        static std::shared_ptr<Data> data;
        int s=-1;                           // starting node
        std::vector<Link> links_;           // links
        double c=0;                         // cost
    public:
        Path(int i) : s{i} {assert(data!=nullptr);}
        Path(std::vector<Link> links, double cost);
        double cost() const {return c;}
        const std::vector<Link>& links() const {return links_;}
        bool loopless() const;
        static void setData(const std::shared_ptr<Data> d) {data=d;}
};

std::ostream& operator<<(std::ostream& os, const Path& p);

#endif

