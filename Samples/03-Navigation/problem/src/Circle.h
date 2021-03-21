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
        float center[2];
        float ray;
        float phaseStart;
        float phaseEnd;
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
        const float phaseDelta;
        
        float phaseCursor;
    };
}

#endif