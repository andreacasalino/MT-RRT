/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_MANAGER_H
#define MT_RRT_TRAJECTORY_MANAGER_H

#include <trajectory/Trajectory.h>

namespace mt::traj {
    class Manager {
    public:
        virtual	~Manager() = default;

        virtual std::unique_ptr<Manager> copy() const = 0;

        virtual float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const = 0;

        virtual TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const = 0;

    protected:
        Manager() = default;
    };

    typedef std::unique_ptr<Manager> ManagerPtr;
}

#endif