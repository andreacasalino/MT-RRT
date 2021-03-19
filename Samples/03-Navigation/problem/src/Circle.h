/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CIRCLE_H
#define MT_RRT_SAMPLE_NAVIGATION_CIRCLE_H

#include <trajectory/Trajectory.h>
#include <Node.h>

namespace mt::traj {
    struct CircleInfo {
        float centerX;
        float centerY;
        float ray;
        float angleStart;
        float angleEnd;
    };

    float cost2Go(const CircleInfo& circleInfo);
    
    class Circle : public Trajectory {
    public:
        Circle(const CircleInfo& info, const float& angleSteer);

        NodeState getCursor() const override;

        inline const Cost& getCumulatedCost() const override { return this->cumulatedCost; };

        AdvanceInfo advance() override;

        inline bool isAntiClockWise() const { return (this->info.angleEnd >= this->info.angleStart); };

    private:
        const CircleInfo info;
        const float angleSteer;

        Cost cumulatedCost;
        float angleCursor;
    };
}

#endif