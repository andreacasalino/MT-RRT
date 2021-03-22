/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/TrajectoryComposite.h>

namespace mt::traj {
    AdvanceInfo TrajectoryComposite::advanceInternal() {
        auto info = (*this->piecesCursor)->advance();
        this->cumulatedCost.set(this->cumulatedCostPrevPieces.get() + (*this->piecesCursor)->getCumulatedCost());
        if(AdvanceInfo::targetReached == info) {
            ++this->piecesCursor;
            if(this->piecesCursor == this->pieces.end()) {
                --this->piecesCursor;
                return AdvanceInfo::targetReached;
            }
            this->cumulatedCostPrevPieces.set(this->cumulatedCost.get());
            info = AdvanceInfo::advanced;
        }
        return info;
    }
}
