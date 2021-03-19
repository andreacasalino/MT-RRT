/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Bubble.h"
#include <Checker.h>
#include <geometry/SphereLogger.h>

namespace mt::traj {
    sample::structJSON BubbleFactory::logDescription() const {
        sample::structJSON result;
        {
            sample::arrayJSON obstacles;
            for (auto it = this->description.obstacles.begin(); it != this->description.obstacles.end(); ++it) {
                obstacles.addElement(log(*it));
            }
            result.addElement("obstacles", obstacles);
        }
        {
            sample::arrayJSON robots;
            for (auto it = this->description.robots.begin(); it != this->description.robots.end(); ++it) {
                sample::arrayJSON temp;
                temp.addElement(sample::Number<float>(it->getBase().x()));
                temp.addElement(sample::Number<float>(it->getBase().y()));
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(sample::Number<float>(l->length.get()));
                }
                for (auto l = it->getLinks().begin(); l != it->getLinks().end(); ++l) {
                    temp.addElement(sample::Number<float>(l->ray.get()));
                }
                robots.addElement(temp);
            }
            result.addElement("robots", robots);

        }
        return result;
    }

    BubbleFactory::BubbleFactory(const sample::Description& description)
        : LineFactory(2 * 3.141f / 180.f)
        , sample::SampleDescription<sample::Description>(description) {
    }

    traj::TrajectoryPtr BubbleFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Bubble>(start, ending_node, this->steerDegree, &this->description);
    }

    Bubble::Bubble(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::Description* data)
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