/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_MULTI_AGENT_STRATEGY_H
#define MT_RRT_MULTI_AGENT_STRATEGY_H

#include <solver/Strategy.h>

namespace mt::solver {
    /** @brief strategy described in METTERE
	 */
    class MultiAgentStrategy : public Strategy {
    public:
        MultiAgentStrategy() = default;

        /** @throw passing Bidir as rrtStrategy
         */
        std::unique_ptr<SolutionInfo> solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) final;

        inline Limited<double>& getIterationsMax() { return this->reallignmentCoeff; };

    private:
        /** @brief refer to METTERE
	     */
        Limited<double> reallignmentCoeff = Limited<double>(0.01, 0.99, 0.05f);
    };
}

#endif