/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "LineWithCheck.h"

namespace mt::traj {
    LineWithCheckManager::LineWithCheckManager(const float& steerDegree, const std::vector<sample::Obstacle>& obstacles)
        : LineManager(steerDegree)
        , obstacles(obstacles) {
    }

    traj::TrajectoryPtr LineWithCheckManager::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<LineWithCheck>(start, ending_node, this->steerDegree, &this->obstacles);
    }

    LineWithCheck::LineWithCheck(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::Obstacle>* obstacles)
        : Line(start, target, steerDegree) {
        this->obstacles = obstacles;
    };

    Trajectory::AdvanceInfo LineWithCheck::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->traj::Line::advance();
        for (auto it = this->obstacles->begin(); it != this->obstacles->end(); ++it) {
            if (it->collideWithSegment(this->cursor.data(), this->previousState.data())) {
                temp = traj::Trajectory::AdvanceInfo::blocked;
                std::swap(this->cursor, this->previousState);
                this->cumulatedCost.set(prevCost);
                break;
            }
        }
        return temp;
    };
}