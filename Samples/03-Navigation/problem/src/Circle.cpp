/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Circle.h"
#include <math.h>
#include <PI.h>

namespace mt::traj {
    float computePhaseDelta(const CircleInfo& info, const float& cartesianSteer) {
        float phaseDelta = info.phaseEnd - info.phaseStart;
        float pieces = ceilf(fabs(phaseDelta) * info.ray / cartesianSteer);
        return  phaseDelta / pieces;
    };

    Circle::Circle(const CircleInfo& info, const float& cartesianSteer)
        : info(info)
        , phaseDelta(computePhaseDelta(info, cartesianSteer)) {
        this->phaseCursor = info.phaseStart;
    }

    NodeState Circle::getCursor() const {
        return NodeState {
            this->info.center[0] + this->info.ray * cosf(this->phaseCursor),
            this->info.center[1] + this->info.ray * sinf(this->phaseCursor),
            this->getOrientation()
        };
    }

    float Circle::getOrientation() const {
        float orientation = this->phaseCursor;
        if(this->info.phaseEnd > this->info.phaseStart) {
            orientation += mt::sample::C_PI_2;
        }
        else {
            orientation -= mt::sample::C_PI_2;
        }
        return atan2(sinf(orientation), cosf(orientation) );
    }

    traj::AdvanceInfo Circle::advanceInternal() {
        float delta = fabs(this->info.phaseEnd - this->phaseCursor);
        if(delta < fabs(this->phaseDelta)) {
            this->phaseCursor = this->info.phaseEnd;
            this->cumulatedCost.set(fabs(this->phaseCursor - this->info.phaseStart) * this->info.ray);
            return traj::AdvanceInfo::targetReached;
        }
        this->phaseCursor += this->phaseDelta;
        this->cumulatedCost.set(fabs(this->phaseCursor - this->info.phaseStart) * this->info.ray);
        return traj::AdvanceInfo::advanced;
    }
}
