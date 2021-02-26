/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_BIDIR_H
#define MT_RRT_EXTENDER_BIDIR_H

#include "Extender.h"
#include <tuple>

namespace mt {
    typedef std::tuple<const Node*, const Node*, float> BidirSolution;
    inline bool operator<(const BidirSolution& a, const BidirSolution& b) {
        return (std::get<2>(a) < std::get<2>(b));
    };

    class ExtBidir : public Extender<BidirSolution> {
    public:
        ExtBidir(const bool& cumulateSolutions, const double& deterministicCoefficient, Tree& leftTree, Tree& rightTree);

        void extend(const size_t& Iterations) override;

    private:
        std::vector<NodeState> computeSolutionSequence(const BidirSolution& sol) const override;

        Tree& leftTree;
        Tree& rightTree;
    };
}

#endif