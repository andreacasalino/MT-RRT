/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Circle.h"

namespace mt::traj {
    Circle::Circle(const CircleInfo& info, const float& angleSteer)
        : info(info)
        , angleSteer(angleSteer) {
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

    traj::Trajectory::AdvanceInfo Circle::advance() {
        float angleDelta = this->info.angleEnd - this->angleCursor;
        if(fabs(angleDelta) < this->angleSteer) {
            this->angleCursor = this->info.angleEnd;
            return traj::Trajectory::AdvanceInfo::targetReached;
        }
        this->angleCursor += angleDelta;
        return traj::Trajectory::AdvanceInfo::advanced;
    }
}
