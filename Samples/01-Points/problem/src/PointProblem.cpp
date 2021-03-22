/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PointProblem.h>
#include <sampler/HyperBox.h>
#include "LineWithCheck.h"

namespace mt::sample {
    sampling::SamplerPtr make_sampler(const geometry::Rectangle& boundaries) {
        NodeState low = { boundaries.getXMin() , boundaries.getYMin() };
        NodeState upp = { boundaries.getXMax() , boundaries.getYMax() };
        return std::make_unique<sampling::HyperBox>(low, upp);
    }

    ProblemPtr makeProblemPoint(const geometry::Rectangle& boundaries, const std::vector<geometry::Rectangle>& obstacles) {
        Description descr = {boundaries, obstacles};
        return std::make_unique<Problem>(make_sampler(descr.boundaries),
                                         std::make_unique<traj::LineWithCheckFactory>(descr),
                                         2, 500.f);
    }
}
