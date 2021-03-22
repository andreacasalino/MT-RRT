/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/TrajectoryBase.h>
#include <Error.h>

namespace mt::traj {
    AdvanceInfo TrajectoryBase::advance() {
        if(AdvanceInfo::advanced != this->lastAdvanceResult) {
            throw Error("This trajectory can't be advanced further");
        }
        this->lastAdvanceResult = this->advanceInternal();
        return this->lastAdvanceResult;
    }
}
