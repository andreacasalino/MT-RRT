/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_QUERY_PARALLELIZED_STRATEGY_H
#define MT_RRT_QUERY_PARALLELIZED_STRATEGY_H

#include <solver/Strategy.h>

namespace mt::solver {
    /** @brief strategy described in METTERE
	 */
    class QueryParallStrategy : public Strategy {
    public:
        QueryParallStrategy() = default;

        std::unique_ptr<SolutionInfo> solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) final;
    };
}

#endif