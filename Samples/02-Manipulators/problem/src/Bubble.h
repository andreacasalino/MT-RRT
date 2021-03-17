/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_BUBBLE_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_BUBBLE_H

#include <trajectory/Line.h>
#include <ManipulatorProblem.h>

namespace mt::traj {
    class BubbleManager : public traj::LineManager {
    public:
        BubbleManager(const float& steerDegree, const sample::ProblemData& data);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        inline std::unique_ptr<TrajectoryManager> copy() const override { return std::make_unique<BubbleManager>(this->steerDegree, this->data); };

        inline const sample::ProblemData& getData() { return this->data; };

    private:
        const sample::ProblemData data;
    };

    class Bubble : public traj::Line {
    public:
        Bubble(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::ProblemData* data);

        AdvanceInfo advance() override;

    private:
        const sample::ProblemData* data;
    };
}

#endif