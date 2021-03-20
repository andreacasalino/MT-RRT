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
        if(AdvanceInfo::blocked == this->lastAdvanceResult) {
            throw Error("A blocked trajectory can't be advanced");
        }
        this->lastAdvanceResult = this->advanceInternal();
        return this->lastAdvanceResult;
    }
}
