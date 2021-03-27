/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Tunneled.h"
#include <PI.h>

namespace mt::traj {
    TunneledFactory::TunneledFactory(const sample::Description& description)
        : LineFactory(mt::sample::C_2_PI / 180.f)
        , DescriptionLogger(description) {
    }

    traj::TrajectoryPtr TunneledFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Tunneled>(start, ending_node, this->steerDegree, &this->description, &this->checker);
    }

    Tunneled::Tunneled(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::Description* data, sample::geometry::SegmentPointChecker* checker)
        : Line(start, target, steerDegree)
        , data(data)
        , checker(checker){
    };

    AdvanceInfo Tunneled::advanceInternal() {
        auto temp = this->traj::Line::advanceInternal();
        const float* pose = this->cursor.data();
        std::size_t cursor = 0;
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            auto capsules = it->directKinematics(&pose[cursor]);
            for (auto itO = this->data->obstacles.begin(); itO != this->data->obstacles.end(); ++itO) {
                for (auto itC = capsules.begin(); itC != capsules.end(); ++itC) {
                    this->checker->check(sample::geometry::Segment{*itC->pointA , *itC->pointB }, itO->getCenter());
                    if (this->checker->getDistance() < (itO->getRay() + itC->ray)) {
                        return traj::AdvanceInfo::blocked;
                    }
                }
            }
            cursor += it->dof();
        }
        return temp;
    };
}