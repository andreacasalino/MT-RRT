/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_H
#define MT_RRT_TRAJECTORY_H

#include <Node.h>
#include <memory>

namespace mt::problem {
    class Trajectory {
    public:
        static const float COST_MAX;

        virtual	~Trajectory() = default;

        inline const NodeState& getCursor() const { return this->cursor; };

        inline float getCummulatedCost() const { return this->cumulatedCost; };

        virtual void advanceCursor() = 0;

        virtual bool isCursorAtEnd() const = 0;

    protected:
        Trajectory(const Node& start, const Node& target);

    // data
        const Node& start;
        const Node& target;

        NodeState cursor;
        float cumulatedCost = COST_MAX;
    };

    typedef std::unique_ptr<Trajectory> TrajectoryPtr;
}

#endif