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
        virtual	~Trajectory() = default;

        virtual const NodeState& getCursor() const = 0;

        virtual void advance() = 0;

        virtual bool eot() const = 0;

    protected:
        Trajectory(const Node& start, const Node& target);

    // data
        const Node& start;
        const Node& target;
    };

    typedef std::unique_ptr<Trajectory> TrajectoryPtr;
}

#endif