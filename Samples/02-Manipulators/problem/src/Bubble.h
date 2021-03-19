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
#include <SampleDescription.h>

namespace mt::traj {
    class BubbleFactory 
        : public traj::LineFactory
        , public sample::SampleDescription<sample::Description> {
    public:
        BubbleFactory(const sample::Description& description);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        sample::structJSON logDescription() const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<BubbleFactory>(this->description); };
    };

    class Bubble : public traj::Line {
    public:
        Bubble(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::Description* data);

        AdvanceInfo advance() override;

    private:
        const sample::Description* data;
    };
}

#endif