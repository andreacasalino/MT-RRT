/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H

#include <trajectory/Trajectory.h>
#include <list>

namespace mt::traj {
    class CartTrajectoryManager {
    public:
    };

    class CartTrajectory : public Trajectory {
    public:
        static TrajectoryPtr make(const NodeState& start, const NodeState& target);
        
        Trajectory::AdvanceInfo advance() override;

    protected:
        CartTrajectory(const NodeState& start, TrajectoryPtr lineStart,TrajectoryPtr circle,TrajectoryPtr lineEnd);
        CartTrajectory(const NodeState& start, TrajectoryPtr line);

        Trajectory::AdvanceInfo advanceNoCheck();

        float sumCosts() const;

        NodeState previousState;
        std::list<TrajectoryPtr> pieces;
        std::list<TrajectoryPtr>::iterator piecesCursor;
        std::list<float> costs;
    };
}

#endif