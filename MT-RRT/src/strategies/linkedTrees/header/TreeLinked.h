/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_LINKED_H
#define MT_RRT_TREE_LINKED_H

#include <TreeCore.h>
#include "ListLinked.h"

namespace mt::solver::linked {
    class TreeLinked
        : public TreeCore
        , public ListLinked<NodePtr> {
    public:
        static std::vector<TreePtr> make_trees(NodePtr root, const std::vector<ProblemPtr>& problems);

        Node* add(NodePtr node) override;

        virtual void gather();

    protected:
        TreeLinked(NodePtr root, Problem& problem);
    };
}

#endif