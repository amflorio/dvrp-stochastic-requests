#ifndef PATHTREE_H
#define PATHTREE_H

class PathTree {
    public:
        std::vector<int> preds;
        PathTree(std::vector<int> prds) : preds{move(prds)} {}
};

#endif

