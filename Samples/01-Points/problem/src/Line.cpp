/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Line.h"

namespace mt::traj {
    LineManager::LineManager(const float& steerDegree, const std::vector<sample::Obstacle>& obstacles)
        : EuclideanManager(steerDegree)
        , obstacles(obstacles) {
    }

    traj::TrajectoryPtr LineManager::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Line>(start, ending_node, this->steerDegree, &this->obstacles);
    }

    Line::Line(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::Obstacle>* obstacles)
        : EuclideanTraj(start, target, steerDegree) {
        this->obstacles = obstacles;
    };

    Trajectory::AdvanceInfo Line::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->traj::EuclideanTraj::advance();
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