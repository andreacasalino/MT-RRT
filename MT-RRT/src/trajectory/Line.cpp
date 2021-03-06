/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/Line.h>
#include <math.h>

namespace mt::traj {
    float euclideanDistance(const float* bufferA, const float* bufferB, const std::size_t& buffersSize) {
        float distance = 0.f;
        for (std::size_t p = 0; p < buffersSize; ++p) {
            distance += powf(bufferA[p] - bufferB[p], 2.f);
        }
        return sqrtf(distance);
    }

    LineFactory::LineFactory(const float& steerDegree)
        : steerDegree(static_cast<float>(fabs(steerDegree))) {
    }

    float LineFactory::cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const {
        return euclideanDistance(start.data(), ending_node.data(), start.size());
    }

    Line::Line(const NodeState& start, const NodeState& target, const float& steerDegree)
        : target(target)
        , steerDegree(steerDegree) {
        this->cursor = start;
    }

    AdvanceInfo Line::advanceInternal() {
        float c = 1.f, delta, catt;
        for (std::size_t k = 0; k<this->target.size() ; ++k) {
            delta = fabsf(this->target[k] - this->cursor[k]);
            if (delta > this->steerDegree) {
                catt = this->steerDegree / delta;
                if (catt < c) {
                    c = catt;
                }
            }
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
    }
}