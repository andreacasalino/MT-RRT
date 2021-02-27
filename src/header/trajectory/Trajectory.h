/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_H
#define MT_RRT_TRAJECTORY_H

#include <Node.h>

namespace mt::traj {
    class Trajectory {
    public:
        static const float COST_MAX;

        virtual	~Trajectory() = default;

        Trajectory(const Trajectory&) = delete;
        Trajectory& operator=(const Trajectory&) = delete;

        inline const NodeState& getCursor() const { return this->cursor; };

        inline float getCummulatedCost() const { return this->cumulatedCost; };

        enum advanceInfo { blocked, advanced, targetReached };

        // return false when the advancement was not possible
        virtual advanceInfo advance() = 0;

    protected:
        Trajectory(const NodeState& start, const NodeState& target);

    // data
        const NodeState& target;

        NodeState cursor;
        float cumulatedCost = COST_MAX;
    };

    typedef std::unique_ptr<Trajectory> TrajectoryPtr;
}

#endif