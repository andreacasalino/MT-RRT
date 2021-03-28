/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H

#include <trajectory/TrajectoryComposite.h>
#include <trajectory/LineTrgSaved.h>
#include "Circle.h"
#include <NavigationProblem.h>
#include <SampleDescription.h>

namespace mt::traj {
    class CartTrajectoryFactory 
        : public TrajectoryFactory 
        , public sample::SampleDescription<sample::Description> {
    public:
        CartTrajectoryFactory(const sample::Description& description);

        TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const;

        sample::structJSON logDescription() const override;

        inline std::unique_ptr<TrajectoryFactory> copy() const override { return std::make_unique<CartTrajectoryFactory>(this->description); };

    private:
        // cache 
        mutable float lastTrajectoryCost2Go;
        float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const;

        const float steerDegree;
    };

    class CartTrajectory : public TrajectoryComposite {
    public:
        CartTrajectory(std::unique_ptr<LineTrgSaved> lineStart,std::unique_ptr<Circle> circle, std::unique_ptr<Line> lineEnd, const sample::Description* data);
        CartTrajectory(std::unique_ptr<Line> line, const sample::Description* data);
        
    protected:        
        AdvanceInfo advanceInternal() override;

        const sample::Description* data;
    };
}

#endif