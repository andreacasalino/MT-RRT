/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CIRCLE_H
#define MT_RRT_SAMPLE_NAVIGATION_CIRCLE_H

#include <trajectory/TrajectoryBase.h>
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
    
    class Circle : public TrajectoryBase {
    public:
        Circle(const CircleInfo& info, const float& cartesianSteer);

        NodeState getCursor() const override;

    private:
        AdvanceInfo advanceInternal() override;
        float getOrientation() const;

        const CircleInfo info;
        const float angleDelta;
        
        float angleCursor;
    };
}

#endif