/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_H
#define MT_RRT_TRAJECTORY_H

#include <Node.h>
#include <trajectory/Cost.h>

namespace mt::traj {
    enum AdvanceInfo { blocked, advanced, targetReached };

    class Trajectory {
    public:
        virtual	~Trajectory() = default;

        Trajectory(const Trajectory&) = delete;
        Trajectory& operator=(const Trajectory&) = delete;

        // spiegare che questi 2 ritornano un numero senza senso se ultimo advance e stato blocked
        virtual NodeState getCursor() const = 0;
        virtual const Cost& getCumulatedCost() const = 0;

        virtual AdvanceInfo advance() = 0;

    protected:
        Trajectory() = default;
    };

    typedef std::unique_ptr<Trajectory> TrajectoryPtr;
}

#endif