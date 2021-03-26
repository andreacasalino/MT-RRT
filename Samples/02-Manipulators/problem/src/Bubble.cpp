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
        return std::make_unique<Bubble>(start, ending_node, &this->description);
    }

    Bubble::Bubble(const NodeState& start, const NodeState& target, const sample::Description* data)
        : Line(start, target, 1.f)
        , data(data) {
    };

    std::vector<float> Bubble::computeRays(const std::vector<sample::Capsule>& links) {
        std::vector<float> rays;
        rays.reserve(links.size());
        std::size_t k2;
        float rTemp;
        for (std::size_t k = 0; k < links.size(); ++k) {
            rays.push_back(0.f);
            for (k2 = k; k2 < links.size(); ++k2) {
                rTemp = euclideanDistance(links[k].pointA->data(), links[k2].pointA->data(), 2) + std::fmax(links[k2].ray, links[k].ray);
                if (rTemp > rays.back()) {
                    rays.back() = rTemp;
                }
            }
        }
        return rays;
    }

    float Bubble::computeMinDist(const std::vector<sample::Capsule>& links) {
        float minDist = MAX_DISTANCE, temp;
        sample::geometry::SegmentPointChecker checker;
        std::size_t l;
        for (auto it = this->data->obstacles.begin(); it != this->data->obstacles.end(); ++it) {
            for (l = 0; l < links.size(); ++l) {
                checker.check(sample::geometry::Segment(*links[l].pointA, *links[l].pointB) , it->getCenter());
                temp = checker.getDistance() - it->getRay() - links[l].ray;
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
        if (ptA->size() >  ptB->size()) {
            std::swap(ptA, ptB);
        }
        float minDist = MAX_DISTANCE, temp;
        sample::geometry::SegmentSegmentChecker checker;
        std::size_t l2;
        for (std::size_t l1 = 0; l1 < ptB->size(); ++l1) {
            for (l2 = l1; l2 < ptA->size(); ++l2) {
                checker.check(sample::geometry::Segment(*(*ptA)[l2].pointA, *(*ptA)[l2].pointB), 
                              sample::geometry::Segment(*(*ptB)[l1].pointA, *(*ptB)[l1].pointB));
                temp = checker.getDistance() - (*ptA)[l2].ray - (*ptB)[l1].ray;
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

    AdvanceInfo Bubble::advanceInternal() {
        this->qDelta = this->target;
        for (std::size_t k = 0; k < qDelta.size(); ++k) {
            qDelta[k] -= this->cursor[k];
            qDelta[k] = fabsf(qDelta[k]);
        }

        float c = 1.f, cTemp;

        std::vector<std::vector<sample::Capsule>> shapes;
        shapes.reserve(this->data->robots.size());
        std::vector<float> raysXDeltaQ;
        raysXDeltaQ.reserve(this->data->robots.size());

        auto dot = [](const float* a, const float* b, const std::size_t& size) {
            float res = a[0] * b[0];
            for (std::size_t k = 1; k < size; ++k) {
                res += a[k] * b[k];
            }
            return res;
        };

        // links vs obstacles
        std::size_t q = 0;
        for (auto it = this->data->robots.begin(); it != this->data->robots.end(); ++it) {
            shapes.emplace_back(it->directKinematics(&this->cursor.data()[q]));
            raysXDeltaQ.emplace_back(dot(this->computeRays(shapes.back()).data(),
                                     &qDelta[q], it->dof()));
            cTemp = this->computeMinDist(shapes.back());
            if (cTemp != MAX_DISTANCE) {
                cTemp = cTemp / raysXDeltaQ.back();
                if (cTemp < c) {
                    c = cTemp;
                }
            }
            q += it->dof();
        }
        // links vs links
        std::size_t r2;
        for (std::size_t r1 = 0; r1 < (this->data->robots.size()-1); ++r1) {
            for (r2 = r1 + 1; r2 < this->data->robots.size(); ++r2) {
                cTemp = this->computeMinDist(shapes[r1], shapes[r2]);
                cTemp = cTemp / (raysXDeltaQ[r1] + raysXDeltaQ[r2]);
                if (cTemp < c) {
                    c = cTemp;
                }
            }
        }

        if (c < ADVANCE_THRESHOLD) {
            return AdvanceInfo::blocked;
        }

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