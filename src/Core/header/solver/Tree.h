/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_H
#define MT_RRT_TREE_H

#include <Node.h>

namespace mt::solver {
    class Tree {
    public:
        virtual	~Tree() = default;
    };

    typedef std::unique_ptr<Tree> TreePtr;
}

#endif