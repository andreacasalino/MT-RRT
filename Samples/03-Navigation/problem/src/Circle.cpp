/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Circle.h"
#include <math.h>

namespace mt::traj {
    float computedDelta(const CircleInfo& info, const float& cartesianSteer) {
        float deltaAngle = info.angleEnd - info.angleStart;
        float pieces = ceilf(fabs(deltaAngle) * info.ray / cartesianSteer);
        return  deltaAngle / pieces;
    };

    Circle::Circle(const CircleInfo& info, const float& cartesianSteer)
        : info(info)
        , angleDelta(computedDelta(info, cartesianSteer)) {
        this->angleCursor = info.angleStart;
    }

    NodeState Circle::getCursor() const {
        return NodeState {
            this->info.centerX + this->info.ray * cosf(this->angleCursor),
            this->info.centerY + this->info.ray * sinf(this->angleCursor),
            this->getOrientation()
        };
    }

    float Circle::getOrientation() const {
        float orientation;
        if(this->info.angleEnd > this->info.angleStart) {
            orientation = M_PI_2;
        }
        else {
            orientation = -M_PI_2;
        }
        orientation += this->angleCursor;
        return atan2(sinf(orientation), cosf(orientation) );
    }

    traj::AdvanceInfo Circle::advanceInternal() {
        float delta = this->info.angleEnd - this->angleCursor;
        if(fabs(delta) < fabs(this->angleDelta)) {
            this->angleCursor = this->info.angleEnd;
            this->cumulatedCost.set(this->cumulatedCost.get() + fabs(delta) * this->info.ray);
            return traj::AdvanceInfo::targetReached;
        }
        this->angleCursor += this->angleDelta;
        this->cumulatedCost.set(this->cumulatedCost.get() + fabs(this->angleDelta) * this->info.ray);
        return traj::AdvanceInfo::advanced;
    }
}
