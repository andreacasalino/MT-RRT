/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_EUCLIDEAN_H
#define MT_RRT_TRAJECTORY_EUCLIDEAN_H

#include <trajectory/TrajectoryManager.h>

namespace mt::traj {
    class EuclideanManager : public TrajectoryManager {
    public:
        float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const override;

    protected:
        EuclideanManager(const float& steerDegree);

        float steerDegree;
    };

    class EuclideanTraj : public Trajectory {
    public:
        EuclideanTraj(const NodeState& start, const NodeState& target, const float& steerDegree);

        AdvanceInfo advance() override;

    protected:
        NodeState previousState;
        float steerDegree;
    };
}

#endif