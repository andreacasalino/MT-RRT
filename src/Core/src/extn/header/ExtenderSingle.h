/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_SINGLE_H
#define MT_RRT_EXTENDER_SINGLE_H

#include "Extender.h"

namespace mt::solver::extn {
    typedef std::pair<const Node*, float> SingleSolution;
    inline bool operator<(const SingleSolution& a, const SingleSolution& b) {
        return (a.second < b.second);
    };

    class Single : public Extender<SingleSolution> {
    public:
        Single(const bool& cumulateSolutions, const double& deterministicCoefficient, tree::Tree& tree, const NodeState& target);

        void extend(const size_t& Iterations) override;

    private:
        std::vector<NodeState> computeSolutionSequence(const SingleSolution& sol) const override;

        tree::Tree& tree;
        NodeState target;
    };
}

#endif