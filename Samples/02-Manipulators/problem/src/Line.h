/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_LINE_H

#include <trajectory/EuclideanManager.h>
#include <ManipulatorProblem.h>

namespace mt::traj {
    class LineManager : public traj::EuclideanManager {
    public:
        LineManager(const float& steerDegree, const sample::ProblemData& data);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        inline std::unique_ptr<TrajectoryManager> copy() const override { return std::make_unique<LineManager>(this->steerDegree, this->data); };

        inline const sample::ProblemData& getData() { return this->data; };

    private:
        const sample::ProblemData data;
    };

    class Line : public traj::EuclideanTraj {
    public:
        Line(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::ProblemData* data);

        AdvanceInfo advance() override;

    private:
        const sample::ProblemData* data;
    };
}

#endif