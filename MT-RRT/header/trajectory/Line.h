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
    /** @brief Computes the euclidean distance of bufferA w.r.t bufferB, both having a size equal to buffersSize
	 */
    float euclideanDistance(const float* stateA, const float* stateB, const std::size_t& buffersSize);
    
    class LineFactory : public TrajectoryFactory  {
    protected:
        LineFactory(const float& steerDegree);

        float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const override;

        const float steerDegree;
    };

    /** @brief Advances along a segment in the state space, traversing everytime a distance not higher than steerDegree
	 */
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