/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_LINKED_H
#define MT_RRT_TREE_STAR_LINKED_H

#include "TreeLinked.h"
#include <TreeRewirer.h>

namespace mt::solver::linked {
    class TreeStarLinked
        : public TreeLinked
        , public TreeRewirer
        , public ListLinked<Rewire> {
    public:
        static std::vector<TreePtr> make_trees(NodePtr root, const std::vector<ProblemPtr>& problems);

        Node* add(NodePtr node) override;

        void gather() override;

    private:
        TreeStarLinked(NodePtr root, Problem& problem);
    };
}

#endif