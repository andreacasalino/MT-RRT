/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_EUCLIDEAN_H
#define MT_RRT_TRAJECTORY_EUCLIDEAN_H

#include <trajectory/Manager.h>

namespace mt::traj {
    class Euclidean : public Manager {
    public:
        float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const override;

        TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

    protected:
        Euclidean(const float& steerDegree);

        class EuclideanTraj;

    private:
        float steerDegree;
    };

    class Euclidean::EuclideanTraj : public Trajectory {
    public:
        EuclideanTraj(const NodeState& start, const NodeState& target, const float& steerDegree);

        advanceInfo advance() override;

    protected:
        NodeState previousState;
        float steerDegree;
    };
}

#endif