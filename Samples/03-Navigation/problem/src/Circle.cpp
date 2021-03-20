/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Circle.h"
#include <math.h>

namespace mt::traj {
    float angularSteer(const CircleInfo& info, const float& cartesianSteer) {
        float deltaAngle = fabs(info.angleEnd - info.angleStart);
        float pieces = ceilf(deltaAngle * info.ray / cartesianSteer);  
        return  deltaAngle / pieces;
    };

    Circle::Circle(const CircleInfo& info, const float& cartesianSteer)
        : info(info)
        , angleSteer(angularSteer(info, cartesianSteer)) {
        this->angleCursor = info.angleStart;
    }

    NodeState Circle::getCursor() const {
        NodeState state;
        state.resize(3);
        state[0] = this->info.centerX + this->info.ray * cosf(this->angleCursor);
        state[1] = this->info.centerY + this->info.ray * sinf(this->angleCursor);
        state[2] = this->angleCursor;
        return state;
    }

    traj::AdvanceInfo Circle::advanceInternal() {
        float angleDelta = this->info.angleEnd - this->angleCursor;
        if(fabs(angleDelta) < this->angleSteer) {
            this->angleCursor = this->info.angleEnd;
            this->cumulatedCost.set(this->cumulatedCost.get() + fabs(angleDelta) * this->info.ray);
            return traj::AdvanceInfo::targetReached;
        }
        this->angleCursor += this->angleSteer * static_cast<float>(angleDelta > 0.0);
        this->cumulatedCost.set(this->cumulatedCost.get() + this->angleSteer * this->info.ray);
        return traj::AdvanceInfo::advanced;
    }
}
