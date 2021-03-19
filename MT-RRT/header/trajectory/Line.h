/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LINE_H
#define MT_RRT_LINE_H

#include <trajectory/TrajectoryFactory.h>

namespace mt::traj {
    class LineFactory : public TrajectoryFactory  {
    protected:
        LineFactory(const float& steerDegree);

        float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const override;

        const float steerDegree;
    };

    class Line : public Trajectory {
    public:
        Line(const NodeState& start, const NodeState& target, const float& steerDegree);

        inline NodeState getCursor() const override { return this->cursor; };

        inline const Cost& getCumulatedCost() const override { return this->cumulatedCost; };

        AdvanceInfo advance() override;

    protected:
        const NodeState& target;
        const float steerDegree;

        NodeState cursor;
        Cost cumulatedCost;

        NodeState previousState;
    };
}

#endif