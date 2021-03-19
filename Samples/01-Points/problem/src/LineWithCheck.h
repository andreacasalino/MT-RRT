/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H

#include <trajectory/Line.h>
#include <Rectangle.h>

namespace mt::traj {
    class LineWithCheckFactory : public traj::LineFactory {
    public:
        LineWithCheckFactory(const float& steerDegree, const std::vector<sample::geometry::Rectangle>& obstacles);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<LineWithCheckFactory>(this->steerDegree, this->obstacles); };

        inline const std::vector<sample::geometry::Rectangle>& getObstacles() const { return this->obstacles; }

    private:
        const std::vector<sample::geometry::Rectangle> obstacles;
    };

    class LineWithCheck : public traj::Line {
    public:
        LineWithCheck(const NodeState& start, const NodeState& target, const float& steerDegree, const std::vector<sample::geometry::Rectangle>* obstacles);

        AdvanceInfo advance() override;

    private:
        const std::vector<sample::geometry::Rectangle>* obstacles;
    };
}

#endif