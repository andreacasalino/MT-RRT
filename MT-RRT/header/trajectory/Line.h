/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LINE_H
#define MT_RRT_LINE_H

#include <trajectory/TrajectoryFactory.h>
#include <trajectory/TrajectoryBase.h>

namespace mt::traj {
    float euclideanDistance(const float* bufferA, const float* bufferB, const std::size_t& buffersSize);
    
    class LineFactory : public TrajectoryFactory  {
    protected:
        LineFactory(const float& steerDegree);

        float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const override;

        const float steerDegree;
    };

    class Line : public TrajectoryBase {
    public:
        Line(const NodeState& start, const NodeState& target, const float& steerDegree);

        inline NodeState getCursor() const override { return this->cursor; };

    protected:
        AdvanceInfo advanceInternal() override;

        const NodeState& target;
        const float steerDegree;

        NodeState cursor;
        NodeState previousState;
    };
}

#endif