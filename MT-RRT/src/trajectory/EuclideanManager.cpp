/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/EuclideanManager.h>
#include <math.h>

namespace mt::traj {
    float squaredDistance(const NodeState& start, const NodeState& ending_node) {
        float distance = 0.f;
        for (std::size_t p = 0; p < start.size(); ++p) {
            distance += powf(start[p] - ending_node[p], 2.f);
        }
        return distance;
    }

    EuclideanManager::EuclideanManager(const float& steerDegree)
        : steerDegree(static_cast<float>(fabs(steerDegree))) {
    }

    float EuclideanManager::cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const {
        float distance = squaredDistance(start, ending_node);
        if (ignoreConstraints) {
            return distance;
        }

        auto line = this->getTrajectory(start, ending_node);
        Trajectory::AdvanceInfo advInfo = Trajectory::AdvanceInfo::advanced;
        while (Trajectory::AdvanceInfo::advanced == advInfo) {
            advInfo = line->advance();
            if (Trajectory::AdvanceInfo::blocked == advInfo) {
                return Trajectory::COST_MAX;
            }
        }
        return distance;
    }

    EuclideanTraj::EuclideanTraj(const NodeState& start, const NodeState& target, const float& steerDegree)
        : Trajectory(start, target)
        , steerDegree(steerDegree) {
        this->cumulatedCost = 0.f;
    }

    Trajectory::AdvanceInfo EuclideanTraj::advance() {
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
        this->cumulatedCost += c * squaredDistance(this->previousState, this->cursor);

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