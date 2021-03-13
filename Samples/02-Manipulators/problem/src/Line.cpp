/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Line.h"
#include <Checker.h>

namespace mt::traj {
    LineManager::LineManager(const float& steerDegree, std::shared_ptr<sample::ProblemData> data)
        : EuclideanManager(steerDegree)
        , data(data) {
    }

    traj::TrajectoryPtr LineManager::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Line>(start, ending_node, this->steerDegree, this->data);
    }

    Line::Line(const NodeState& start, const NodeState& target, const float& steerDegree, std::shared_ptr<sample::ProblemData> data)
        : EuclideanTraj(start, target, steerDegree)
        , data(data) {
    };

    Trajectory::AdvanceInfo Line::advance() {
        float prevCost = this->cumulatedCost.get();
        auto temp = this->traj::EuclideanTraj::advance();
        const float* pose = this->cursor.data();
        std::size_t cursor = 0;
        sample::geometry::SegmentPointChecker checker;
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            auto capsules = it->directKinematics(&pose[cursor]);
            for (auto itO = this->data->obstacles.begin(); itO != this->data->obstacles.end(); ++itO) {
                for (auto itC = capsules.begin(); itC != capsules.end(); ++itC) {
                    checker.check(sample::geometry::Segment{*itC->pointA , *itC->pointB }, itO->getCenter());
                    if (checker.getDistance() < (itO->getRay() + itC->ray)) {
                        temp = traj::Trajectory::AdvanceInfo::blocked;
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