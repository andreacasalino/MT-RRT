/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Line.h"

namespace mt::traj {
    LineManager::LineManager(const float& steerDegree, const std::vector<sample::Box>& obstacles)
        : Euclidean(steerDegree) {
        this->obstacles = std::make_shared<const std::vector<sample::Box>>(obstacles);
    }

    traj::TrajectoryPtr LineManager::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Line>(start, ending_node, this->steerDegree, this->obstacles);
    }

    Line::Line(const NodeState& start, const NodeState& target, const float& steerDegree, std::shared_ptr<const std::vector<sample::Box>> obstacles)
        : EuclideanTraj(start, target, steerDegree) {
        this->obstacles = obstacles;
    };

    Trajectory::advanceInfo Line::advance() {
        auto temp = this->traj::EuclideanTraj::advance();
        bool collide = false;
        for (auto it = this->obstacles->begin(); it != this->obstacles->end(); ++it) {

        }
        if (collide) {
            std::swap(this->cursor, this->previousState);
            return traj::Trajectory::advanceInfo::blocked;
        }
        return temp;
    };
}