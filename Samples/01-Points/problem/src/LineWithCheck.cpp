/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "LineWithCheck.h"

namespace mt::traj {
    LineWithCheckFactory::LineWithCheckFactory(const float& steerDegree, const std::vector<sample::geometry::Rectangle>& obstacles)
        : LineFactory(steerDegree)
        , obstacles(obstacles) {
    }

    traj::TrajectoryPtr LineWithCheckFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<LineWithCheck>(start, ending_node, this->steerDegree, &this->obstacles);
    }

    LineWithCheck::LineWithCheck(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::geometry::Rectangle>* obstacles)
        : Line(start, target, steerDegree) {
        this->obstacles = obstacles;
    };

    AdvanceInfo LineWithCheck::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->traj::Line::advance();
        for (auto it = this->obstacles->begin(); it != this->obstacles->end(); ++it) {
            if (it->collideWithSegment(this->cursor.data(), this->previousState.data())) {
                temp = traj::AdvanceInfo::blocked;
                std::swap(this->cursor, this->previousState);
                this->cumulatedCost.set(prevCost);
                break;
            }
        }
        return temp;
    };
}