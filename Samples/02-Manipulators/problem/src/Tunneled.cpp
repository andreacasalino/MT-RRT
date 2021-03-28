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
        return std::make_unique<Tunneled>(start, ending_node, this->steerDegree, &this->description, &this->checkerPoint, &this->checkerSegment);
    }

    Tunneled::Tunneled(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::Description* data,
        sample::geometry::SegmentPointChecker* checkerPoint, sample::geometry::SegmentSegmentChecker* checkerSegment)
        : Line(start, target, steerDegree)
        , data(data)
        , checkerPoint(checkerPoint)
        , checkerSegment(checkerSegment) {
    };

    bool Tunneled::checkCollision(const std::vector<sample::Capsule>& linksA, const std::vector<sample::Capsule>& linksB) {
        const std::vector<sample::Capsule>* ptA = &linksA;
        const std::vector<sample::Capsule>* ptB = &linksB;
        if (ptB->size() < ptA->size()) {
            std::swap(ptA, ptB);
        }
        std::size_t l2;
        for (std::size_t l1 = 0; l1 < ptA->size(); ++l1) {
            for (l2 = l1; l2 < ptB->size(); ++l2) {
                this->checkerSegment->check(sample::geometry::Segment(*(*ptA)[l1].pointA, *(*ptA)[l1].pointB),
                    sample::geometry::Segment(*(*ptB)[l2].pointA, *(*ptB)[l2].pointB));
                if (this->checkerSegment->getDistance() < ((*ptA)[l1].ray + (*ptB)[l2].ray)) {
                    return true;
                }
            }
        }
        return false;
    }

    AdvanceInfo Tunneled::advanceInternal() {
        auto temp = this->traj::Line::advanceInternal();
        const float* pose = this->cursor.data();
        std::size_t cursor = 0;

        std::vector<std::vector<sample::Capsule>> shapes;
        shapes.reserve(this->data->robots.size());

        // links vs obstacles
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            shapes.emplace_back(it->directKinematics(&pose[cursor]));
            for (auto itO = this->data->obstacles.begin(); itO != this->data->obstacles.end(); ++itO) {
                for (auto itC = shapes.back().begin(); itC != shapes.back().end(); ++itC) {
                    this->checkerPoint->check(sample::geometry::Segment{ *itC->pointA , *itC->pointB }, itO->getCenter());
                    if (this->checkerPoint->getDistance() < (itO->getRay() + itC->ray)) {
                        return traj::AdvanceInfo::blocked;
                    }
                }
            }
            cursor += it->dof();
        }

        if (this->data->robots.size() > 1) {
            // links vs links
            std::size_t r2;
            for (std::size_t r1 = 0; r1 < (shapes.size() - 1); ++r1) {
                for (r2 = r1 + 1; r2 < shapes.size(); ++r2) {
                    if (this->checkCollision(shapes[r1], shapes[r2])) {
                        return traj::AdvanceInfo::blocked;
                    }
                }
            }
        }

        return temp;
    };
}