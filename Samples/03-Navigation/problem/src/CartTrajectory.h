/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H

#include <trajectory/Trajectory.h>

namespace mt::traj {
    class CartTrajectoryManager {
    public:
    };

    class CartTrajectory : public Trajectory {
    public:
        static TrajectoryPtr make();
        
        Trajectory::AdvanceInfo advance() override;

    protected:
        CartTrajectory(const NodeState& start, const NodeState& target);
    };
}

#endif