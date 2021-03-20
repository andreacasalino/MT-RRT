/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/TrajectoryComposite.h>

namespace mt::traj {
    AdvanceInfo TrajectoryComposite::advance() {
        auto info = (*this->piecesCursor)->advance();
        if(AdvanceInfo::targetReached == info) {
            ++this->piecesCursor;
            this->cumulatedCost.set(this->cumulatedCost.get() + (*this->piecesCursor)->getCumulatedCost());
            if(this->piecesCursor == this->pieces.end()) {
                --this->piecesCursor;
                return AdvanceInfo::targetReached;
            }
        }
        return info;
    }
}
