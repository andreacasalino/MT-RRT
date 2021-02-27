/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/Euclidean.h>
#include <math.h>

namespace mt::traj {
    float squaredDistance(const NodeState& start, const NodeState& ending_node) {
        float distance = 0.f;
        for (std::size_t p = 0; p < start.size(); ++p) {
            distance += powf(start[p] - ending_node[p], 2.f);
        }
        return distance;
    }

    Euclidean::Euclidean(const float& steerDegree)
        : steerDegree(static_cast<float>(fabs(steerDegree))) {
    }

    float Euclidean::cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const {
        float distance = squaredDistance(start, ending_node);
        if (ignoreConstraints) {
            return distance;
        }

        auto line = this->getTrajectory(start, ending_node);
        Trajectory::advanceInfo advInfo = Trajectory::advanceInfo::advanced;
        while (Trajectory::advanceInfo::advanced == advInfo) {
            advInfo = line->advance();
            if (Trajectory::advanceInfo::blocked == advInfo) {
                return Trajectory::COST_MAX;
            }
        }
        return distance;
    }

    EuclideanTraj::EuclideanTraj(const NodeState& start, const NodeState& target, const float& steerDegree)
        : Trajectory(start, target)
        , steerDegree(steerDegree) {
        this->cumulatedCost = 0.f;
        this->previousState = start;
    }

    Trajectory::advanceInfo EuclideanTraj::advance() {
        float distance = squaredDistance(this->cursor, this->target);
        std::swap(this->previousState, this->cursor);
        if (distance <= this->steerDegree) {
            this->cursor = this->target;
            return advanceInfo::targetReached;
        }
        float c = this->steerDegree / distance;
        float c2 = 1.f - c;
        for (std::size_t k = 0; k < this->cursor.size(); ++k) {
            this->cursor[k] *= c2;
            this->cursor[k] += c * this->target[k];
        }
        return advanceInfo::advanced;
    }
}