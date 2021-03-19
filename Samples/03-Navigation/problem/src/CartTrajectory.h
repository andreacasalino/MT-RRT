/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_TRAJECTORY_H

#include <trajectory/Trajectory.h>
#include "Line2.h"
#include "Circle.h"
#include <NavigationProblem.h>
#include <SampleDescription.h>
#include <list>

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
        const float steerDegree;
    };

    class CartTrajectory : public Trajectory {
    public:
        CartTrajectory(std::unique_ptr<Line2> lineStart,std::unique_ptr<Circle> circle, std::unique_ptr<Line> lineEnd, const sample::Description* data);
        CartTrajectory(std::unique_ptr<Line> line, const sample::Description* data);
        
        inline NodeState getCursor() const override { return (*this->piecesCursor)->getCursor(); };

        inline const Cost& getCumulatedCost() const override { return this->cumulatedCost; };
        
        AdvanceInfo advance() override;

    protected:
        AdvanceInfo CartTrajectory::advanceNoCheck();
        float CartTrajectory::sumCosts() const;
        
        const sample::Description* data;
        
        std::list<TrajectoryPtr> pieces;
        std::list<TrajectoryPtr>::iterator piecesCursor;

        Cost cumulatedCost;
        std::list<float> cumulatedCostContributions;
    };
}

#endif