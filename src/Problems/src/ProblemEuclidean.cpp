/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ProblemEuclidean.h>
#include <math.h>

namespace mt::problem {
    float squaredDistance(const NodeState& start, const NodeState& ending_node) {
        float distance = 0.f;
        for (std::size_t p = 0; p < start.size(); ++p) {
            distance += powf(start[p] - ending_node[p], 2.f);
        }
        return distance;
    }

    class LinearTrajectory : public Trajectory {
    public:
        LinearTrajectory(const NodeState& start, const NodeState& target, const float& steerDegree)
            : Trajectory(start, target)
            , steerDegree(steerDegree) {
            this->cursor = this->start;
            this->cumulatedCost = 0.f;
            this->advanceCursor();
        };

        void advanceCursor() override {
            float distance = squaredDistance(this->cursor, this->target);
            if (distance <= this->steerDegree) {
                this->cursor = this->target;
                this->eot = true;
            }
            float c = this->steerDegree / distance;
            float c2 = 1.f - c;
            for (std::size_t k = 0; k < this->cursor.size(); ++k) {
                this->cursor[k] *= c2;
                this->cursor[k] += c*this->target[k];
            }
        };

        inline bool isCursorAtEnd() const override { return this->eot; };

    private:
        const float steerDegree;
        bool eot = false;
    };

    ProblemEuclidean::ProblemEuclidean(SamplerPtr sampler, CheckerPtr checker, const std::size_t& stateSpaceSize, const float& gamma, const float& steerDegree)
        : Problem(std::move(sampler), std::move(checker), stateSpaceSize, gamma, true)
        , steerDegree(abs(steerDegree)) {
    }

    TrajectoryPtr ProblemEuclidean::getTrajectory(const NodeState& start, const NodeState& trg) {
        return std::make_unique<LinearTrajectory>(start, trg, this->steerDegree);
    }

    float ProblemEuclidean::cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) {
        float distance = squaredDistance(start, ending_node);
        if (ignoreConstraints) {
            return distance;
        }

        LinearTrajectory line(start, ending_node, this->steerDegree);
        while (!line.isCursorAtEnd()) {
            if (this->checker->isNotAdmitted(line.getCursor())) return Trajectory::COST_MAX;
            line.advanceCursor();
        }
        return distance;
    }
}