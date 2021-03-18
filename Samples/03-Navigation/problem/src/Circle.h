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
#include <math.h>

namespace mt::traj {
    struct CircleInfo {
        float centerX;
        float centerY;
        float ray;
        float angleStart;
        float angleEnd;
    };

    float cost2Go(const CircleInfo& circleInfo);
    
    class Circle {
    public:
        Circle(const CircleInfo& info, const float& angleSteer);

        NodeState getCursor() const;

        traj::Trajectory::AdvanceInfo advance();

        float getCumulatedCost() const { return this->info.ray * fabs(this->angleCursor - this->info.angleStart); };

        inline bool isAntiClockWise() const { return (this->info.angleEnd >= this->info.angleStart); };

    private:
        const CircleInfo info;
        const float angleSteer;
        float angleCursor;
    };
}

#endif