/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SERIAL_STRATEGY_H
#define MT_RRT_SERIAL_STRATEGY_H

#include <solver/Strategy.h>

namespace mt::solver {
    /** @brief The standard mono-thread rrt 
	 */
    class SerialStrategy : public Strategy {
    public:
        SerialStrategy() = default;

        std::unique_ptr<SolutionInfo> solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) final;
    };
}

#endif