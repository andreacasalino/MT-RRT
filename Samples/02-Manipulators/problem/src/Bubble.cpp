/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Bubble.h"
#include <Checker.h>
#include <PI.h>
#include <limits>

namespace mt::traj {
    constexpr float MAX_DISTANCE = std::numeric_limits<float>::max();
    constexpr float ADVANCE_THRESHOLD = 0.001f;

    BubbleFactory::BubbleFactory(const sample::Description& description)
        : DescriptionLogger(description) {
    }

    traj::TrajectoryPtr BubbleFactory::getTrajectory(const NodeState& start, const NodeState& ending_node) const {
        return std::make_unique<Bubble>(start, ending_node, &this->description, &this->checkerPoint, &this->checkerSegment);
    }

    Bubble::Bubble(const NodeState& start, const NodeState& target, const sample::Description* data,
        sample::geometry::SegmentPointChecker* checkerPoint, sample::geometry::SegmentSegmentChecker* checkerSegment)
        : Line(start, target, 1.f)
        , data(data)
        , checkerPoint(checkerPoint)
        , checkerSegment(checkerSegment) {
    };

    float getFurthest(const sample::Capsule& cap, const sample::geometry::Point& origin) {
        float dist1 = euclideanDistance(cap.pointA->data(), origin.data(), 2) + cap.ray;
        float dist2 = euclideanDistance(cap.pointB->data(), origin.data(), 2) + cap.ray;
        if (dist2 < dist1) {
            return dist1;
        }
        return dist2;
    }

    std::vector<float> Bubble::computeRays(const std::vector<sample::Capsule>& links) {
        std::vector<float> rays;
        rays.reserve(links.size());
        std::size_t k2;
        float rTemp;
        for (std::size_t k = 0; k < links.size(); ++k) {
            rays.push_back(euclideanDistance(links[k].pointA->data(), links[k].pointB->data(), 2) + links[k].ray);
            for (k2 = k + 1; k2 < links.size(); ++k2) {
                rTemp = getFurthest(links[k2] , *links[k].pointA);
                if (rTemp > rays.back()) {
                    rays.back() = rTemp;
                }
            }
        }
        return rays;
    }

    float Bubble::computeMinDist(const std::vector<sample::Capsule>& links) {
        float minDist = MAX_DISTANCE, temp;
        std::size_t l;
        for (auto it = this->data->obstacles.begin(); it != this->data->obstacles.end(); ++it) {
            for (l = 0; l < links.size(); ++l) {
                this->checkerPoint->check(sample::geometry::Segment(*links[l].pointA, *links[l].pointB) , it->getCenter());
                temp = this->checkerPoint->getDistance() - it->getRay() - links[l].ray;
                if (temp <= 0.f) {
                    return 0.f;
                }
                if (temp < minDist) {
                    minDist = temp;
                }
            }
        }
        return minDist;
    }

    float Bubble::computeMinDist(const std::vector<sample::Capsule>& linksA, const std::vector<sample::Capsule>& linksB) {
        const std::vector<sample::Capsule>* ptA = &linksA;
        const std::vector<sample::Capsule>* ptB = &linksB;
        if (ptB->size() <  ptA->size()) {
            std::swap(ptA, ptB);
        }
        float minDist = MAX_DISTANCE, temp;
        std::size_t l2;
        for (std::size_t l1 = 0; l1 < ptA->size(); ++l1) {
            for (l2 = l1; l2 < ptB->size(); ++l2) {
                this->checkerSegment->check(sample::geometry::Segment(*(*ptA)[l1].pointA, *(*ptA)[l1].pointB),
                    sample::geometry::Segment(*(*ptB)[l2].pointA, *(*ptB)[l2].pointB));
                temp = this->checkerSegment->getDistance() - (*ptA)[l1].ray - (*ptB)[l2].ray;
                if (temp <= 0.f) {
                    return 0.f;
                }
                if (temp < minDist) {
                    minDist = temp;
                }
            }
        }
        return minDist;
    }

    float dot(const float* a, const float* b, const std::size_t& size) {
        float res = a[0] * b[0];
        for (std::size_t k = 1; k < size; ++k) {
            res += a[k] * b[k];
        }
        return res;
    };

    AdvanceInfo Bubble::advanceInternal() {
        this->qDelta = this->target;
        for (std::size_t k = 0; k < qDelta.size(); ++k) {
            qDelta[k] -= this->cursor[k];
            qDelta[k] = fabsf(qDelta[k]);
        }

        float c = 1.f, cTemp;

        std::vector<std::vector<sample::Capsule>> shapes;
        std::vector<float> raysXDeltaQ;
        shapes.reserve(this->data->robots.size());
        raysXDeltaQ.reserve(this->data->robots.size());

        // links vs obstacles
        std::size_t q = 0;
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            shapes.emplace_back(it->directKinematics(&this->cursor.data()[q]));
            auto rays = this->computeRays(shapes.back());
            raysXDeltaQ.emplace_back(dot(rays.data(), &qDelta[q], it->dof()));
            if (!this->data->obstacles.empty()) {
                cTemp = this->computeMinDist(shapes.back());
                cTemp = cTemp / raysXDeltaQ.back();
                if (cTemp < c) {
                    c = cTemp;
                }
            }
            q += it->dof();
        }
        if (this->data->robots.size() > 1) {
            // links vs links
            std::size_t r2;
            for (std::size_t r1 = 0; r1 < (this->data->robots.size() - 1); ++r1) {
                for (r2 = r1 + 1; r2 < this->data->robots.size(); ++r2) {
                    cTemp = this->computeMinDist(shapes[r1], shapes[r2]);
                    cTemp = cTemp / (raysXDeltaQ[r1] + raysXDeltaQ[r2]);
                    if (cTemp < c) {
                        c = cTemp;
                    }
                }
            }
        }

        if (c < ADVANCE_THRESHOLD) {
            return AdvanceInfo::blocked;
        }

        std::swap(this->previousState, this->cursor);
        this->cursor = this->target;
        this->cumulatedCost.set(this->cumulatedCost.get() + c * euclideanDistance(this->previousState.data(), this->cursor.data(), this->cursor.size()));

        if (1.f == c) {
            return AdvanceInfo::targetReached;
        }

        float c2 = 1.f - c;
        for (std::size_t k = 0; k < this->cursor.size(); ++k) {
            this->cursor[k] *= c;
            this->cursor[k] += c2 * this->previousState[k];
        }
        return AdvanceInfo::advanced;
    };
}