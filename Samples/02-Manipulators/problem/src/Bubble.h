/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_BUBBLE_H
#define MT_RRT_SAMPLE_MANIPULATOR_BUBBLE_H

#include <trajectory/Line.h>
#include <Checker.h>
#include "DescriptionLogger.h"

namespace mt::traj {
    class BubbleFactory
        : public TrajectoryFactory
        , public sample::DescriptionLogger {
    public:
        BubbleFactory(const sample::Description& description);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<BubbleFactory>(this->description); };

        inline float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const override {
            return euclideanDistance(start.data(), ending_node.data(), start.size());
        };

    private:
        mutable sample::geometry::SegmentPointChecker checkerPoint;
        mutable sample::geometry::SegmentSegmentChecker checkerSegment;
    };

    class Bubble : public traj::Line {
    public:
        Bubble(const NodeState& start, const NodeState& target, const sample::Description* data,
            sample::geometry::SegmentPointChecker* checkerPoint, sample::geometry::SegmentSegmentChecker* checkerSegment);

    private:
        AdvanceInfo advanceInternal() override;

        std::vector<float> computeRays(const std::vector<sample::Capsule>& links);

        float computeMinDist(const std::vector<sample::Capsule>& links);

        float computeMinDist(const std::vector<sample::Capsule>& linksA, const std::vector<sample::Capsule>& linksB);

        const sample::Description* data;

        NodeState qDelta;
        sample::geometry::SegmentPointChecker* checkerPoint;
        sample::geometry::SegmentSegmentChecker* checkerSegment;
    };
}

#endif