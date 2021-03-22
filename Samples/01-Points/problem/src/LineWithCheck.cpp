/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "LineWithCheck.h"
#include <geometry/RectangleLogger.h>

namespace mt::traj {
    sample::structJSON LineWithCheckFactory::logDescription() const {
        sample::structJSON result;

        result.addElement("limits", sample::geometry::log(this->description.boundaries));

        sample::arrayJSON obstacles;
        for (auto it = this->description.obstacles.begin(); it != this->description.obstacles.end(); ++it) {
            obstacles.addElement(sample::geometry::log(*it));
        }
        result.addElement("obstacles", obstacles);

        return result;
    }

    float getSteerDegree(const sample::geometry::Rectangle& boundaries) {
        float temp1 = 0.05f * (boundaries.getXMax() - boundaries.getXMin());
        float temp2 = 0.05f * (boundaries.getYMax() - boundaries.getYMin());
        if (temp1 < temp2) return temp1;
        return temp2;
    }

    LineWithCheckFactory::LineWithCheckFactory(const sample::Description& description)
        : LineFactory(getSteerDegree(description.boundaries))
        , sample::SampleDescription<sample::Description>(description) {
    }

    traj::TrajectoryPtr LineWithCheckFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<LineWithCheck>(start, ending_node, this->steerDegree, &this->description.obstacles);
    }

    LineWithCheck::LineWithCheck(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::geometry::Rectangle>* obstacles)
        : Line(start, target, steerDegree) {
        this->obstacles = obstacles;
    };

    AdvanceInfo LineWithCheck::advanceInternal() {
        auto temp = this->traj::Line::advanceInternal();
        for (auto it = this->obstacles->begin(); it != this->obstacles->end(); ++it) {
            if (it->collideWithSegment(this->cursor.data(), this->previousState.data())) {
                return traj::AdvanceInfo::blocked;
            }
        }
        return temp;
    };
}