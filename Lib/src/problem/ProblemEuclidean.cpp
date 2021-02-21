/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <problem/ProblemEuclidean.h>
#include <float.h>

namespace mt::problem {
    class LinearTrajectory : public Trajectory {
    public:
        LinearTrajectory(const Node& start, const Node& target);

        const NodeState& getCursor() const override;

        void advance() override;

        bool eot() const override;
    };

    float ProblemEuclidean::cost2Go(const Node& start, const Node& ending_node, const bool& ignoreConstraints) {
        float distance = 0.f;
        for (std::size_t p = 0; p < start.getState().size(); ++p) {
            distance += powf(start.getState()[p] - ending_node.getState()[p], 2.f);
        }
        if (ignoreConstraints) {
            return distance;
        }

        LinearTrajectory line(start, ending_node);
        while (!line.eot()) {
            if (this->checker->isNotAdmitted(line.getCursor())) return FLT_MAX;
            line.advance();
        }
        return distance;
    }
}