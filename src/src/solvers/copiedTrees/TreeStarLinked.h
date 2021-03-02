/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_LINKED_H
#define MT_RRT_TREE_STAR_LINKED_H

#include "TreeConcreteLinked.h"

namespace mt::copied {
    class TreeStarLinked
        : public TreeConcreteLinked
        , public ListLinked<TreeConcrete::Rewird> {
    public:
        static std::vector<TreeStarLinked> make_trees(const std::vector<ProblemPtr>& problems, NodePtr root);

        void add(NodePtr node) override;

        void gather() override;

        std::pair<NodePtr, bool> extend(const NodeState& target) override;

    private:
        TreeStarLinked(Problem& problem, NodePtr root);
    };
}

#endif