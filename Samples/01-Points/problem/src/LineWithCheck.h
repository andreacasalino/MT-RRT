/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H

#include <trajectory/Line.h>
#include <PointProblem.h>
#include <SampleDescription.h>

namespace mt::traj {
    class LineWithCheckFactory 
        : public traj::LineFactory
        , public sample::SampleDescription<sample::Description> {
    public:
        LineWithCheckFactory(const sample::Description& description);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        sample::structJSON logDescription() const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<LineWithCheckFactory>(this->description); };
    };

    class LineWithCheck : public traj::Line {
    public:
        LineWithCheck(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::geometry::Rectangle>* obstacles);

    private:
        AdvanceInfo advanceInternal() override;

        const std::vector<sample::geometry::Rectangle>* obstacles;
    };
}

#endif