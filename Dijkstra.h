#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "Data.h"
#include "Path.h"
#include "PathTree.h"

class Dijkstra {
    private:
        static std::shared_ptr<Data> data;
        static std::vector<PathTree> pathtrees;
        static std::vector<std::vector<double>> costs;
    public:
        Dijkstra();
        static double cost(int s, int t);
        static void setData(const std::shared_ptr<Data> d) {data=d;}
        static Path solve(int s, int t);
        static Path solvest(int s, int t);
        static PathTree solve(int s);
};

#endif

