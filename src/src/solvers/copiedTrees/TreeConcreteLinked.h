/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONCRETE_LINKED_H
#define MT_RRT_TREE_CONCRETE_LINKED_H

#include <TreeConcrete.h>
#include "ListLinked.h"

namespace mt::copied {
    class TreeConcreteLinked
        : public TreeConcrete
        , public ListLinked<NodePtr> {
    public:
        static std::vector<TreePtr> make_trees(const std::vector<ProblemPtr>& problems, NodePtr root);

        void add(NodePtr node) override;

        virtual void gather();

    protected:
        TreeConcreteLinked(Problem& problem, NodePtr root);
    };
}

#endif