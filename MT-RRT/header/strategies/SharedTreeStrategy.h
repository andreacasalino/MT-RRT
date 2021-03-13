/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SHARED_TREE_STRATEGY_H
#define MT_RRT_SHARED_TREE_STRATEGY_H

#include <solver/Strategy.h>

namespace mt::solver {
    class SharedTreeStrategy : public Strategy {
    public:
        SharedTreeStrategy() = default;

        std::unique_ptr<SolutionInfo> solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) final;
    };
}

#endif