/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LINE_H
#define MT_RRT_LINE_H

#include <trajectory/TrajectoryManager.h>

namespace mt::traj {
    class LineManager : public TrajectoryManager {
    public:
        float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const override;

    protected:
        LineManager(const float& steerDegree);

        float steerDegree;
    };

    class Line : public Trajectory {
    public:
        Line(const NodeState& start, const NodeState& target, const float& steerDegree);

        AdvanceInfo advance() override;

    protected:
        const NodeState& target;
        NodeState previousState;
        float steerDegree;
    };
}

#endif