/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONTAINER_H
#define MT_RRT_TREE_CONTAINER_H

#include "../../ProblemBattery.h"
#include "../header/TreeStarLinked.h"

namespace mt::solver::linked {
    class TreeContainer
        : public Tree {
    public:
        TreeContainer(NodePtr root, const std::vector<ProblemPtr>& problems);

        inline Nodes::const_reverse_iterator rend() const override { return this->contained.front()->rend(); };
		inline Nodes::const_reverse_iterator rbegin() const override { return this->contained.front()->rbegin(); };

        void gather();

        inline std::size_t size() const { return this->contained.size(); };

        inline TreeLinked* getContained(const std::size_t& pos) const { return this->contained[pos].get(); };

    protected:
        TreeContainer() = default;

        std::vector<std::unique_ptr<TreeLinked>> contained;
    };

    class TreeStarContainer : public TreeContainer {
    public:
        TreeStarContainer(NodePtr root, const std::vector<ProblemPtr>& problems);
    };
}

#endif