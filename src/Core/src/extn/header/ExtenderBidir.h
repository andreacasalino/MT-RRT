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

namespace mt::solver::extn {
    typedef std::tuple<const Node*, const Node*, float> BidirSolution;

    class Bidir : public Extender<BidirSolution> {
    public:
        Bidir(const bool& cumulateSolutions, const float& deterministicCoefficient, tree::Tree& leftTree, tree::Tree& rightTree);

        void extend(const size_t& Iterations) override;

    private:
        tree::Tree& leftTree;
        tree::Tree& rightTree;
    };
}

#endif