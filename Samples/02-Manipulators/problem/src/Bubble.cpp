/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Bubble.h"
#include <Checker.h>

namespace mt::traj {
    BubbleFactory::BubbleFactory(const float& steerDegree, const sample::ProblemData& data)
        : LineFactory(steerDegree)
        , data(data) {
    }

    traj::TrajectoryPtr BubbleFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Bubble>(start, ending_node, this->steerDegree, &this->data);
    }

    Bubble::Bubble(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::ProblemData* data)
        : Line(start, target, steerDegree)
        , data(data) {
    };

    AdvanceInfo Bubble::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->traj::Line::advance();
        const float* pose = this->cursor.data();
        std::size_t cursor = 0;
        sample::geometry::SegmentPointChecker checker;
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            auto capsules = it->directKinematics(&pose[cursor]);
            for (auto itO = this->data->obstacles.begin(); itO != this->data->obstacles.end(); ++itO) {
                for (auto itC = capsules.begin(); itC != capsules.end(); ++itC) {
                    checker.check(sample::geometry::Segment{*itC->pointA , *itC->pointB }, itO->getCenter());
                    if (checker.getDistance() < (itO->getRay() + itC->ray)) {
                        temp = traj::AdvanceInfo::blocked;
                        std::swap(this->cursor, this->previousState);
                        this->cumulatedCost.set(prevCost);
                        break;
                    }
                }
            }
            cursor += it->dof();
        }
        return temp;
    };
}