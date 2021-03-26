/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_TUNNELED_H
#define MT_RRT_SAMPLE_MANIPULATOR_TUNNELED_H

#include <trajectory/Line.h>
#include "DescriptionLogger.h"

namespace mt::traj {
    class TunneledFactory 
        : public traj::LineFactory
        , public sample::DescriptionLogger {
    public:
        TunneledFactory(const sample::Description& description);

        traj::TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<TunneledFactory>(this->description); };
    };

    class Tunneled : public traj::Line {
    public:
        Tunneled(const NodeState& start, const NodeState& target, const float& steerDegree, const sample::Description* data);

    private:
        AdvanceInfo advanceInternal() override;

        const sample::Description* data;
    };
}

#endif