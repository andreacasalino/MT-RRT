/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_SINGLE_H
#define MT_RRT_EXTENDER_SINGLE_H

#include <TreeCore.h>
#include "Extender.h"

namespace mt {
    typedef std::pair<const Node*, float> SingleSolution;
    inline bool operator<(const SingleSolution& a, const SingleSolution& b) {
        return (a.second < b.second);
    };

    class ExtSingle : public Extender<SingleSolution> {
    public:
        ExtSingle(const bool& cumulateSolutions, const double& deterministicCoefficient, Tree& tree, const NodeState& target);

        void extend(const size_t& Iterations) override;

        std::vector<NodeState> computeSolutionSequence(const SingleSolution& sol) const override;

    private:
        TreeCore& tree;
        NodeState target;
    };

    std::vector<ExtSingle> make_battery(const bool& cumulateSolutions, const double& deterministicCoefficient, const std::vector<TreePtr>& trees, const NodeState& target);
}

#endif