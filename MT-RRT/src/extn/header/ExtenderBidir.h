/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_BIDIR_H
#define MT_RRT_EXTENDER_BIDIR_H

#include <TreeCore.h>
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

        void extend(const std::size_t& Iterations) override;

        std::vector<NodeState> computeSolutionSequence(const BidirSolution& sol) const override;

    private:
        BidirSolution makeSolution(const Node* a, const Node* b, const bool& caso) const;

        TreeCore& leftTree;
        TreeCore& rightTree;
    };

    std::vector<ExtBidir> make_battery(const bool& cumulateSolutions, const double& deterministicCoefficient, const std::vector<TreePtr>& treesA, const std::vector<TreePtr>& treesB);
}

#endif