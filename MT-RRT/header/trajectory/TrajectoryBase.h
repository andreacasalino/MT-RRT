/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_BASE_H
#define MT_RRT_TRAJECTORY_BASE_H

#include <trajectory/Trajectory.h>

namespace mt::traj {
    /** @brief Base class for a Trajectory. Calling advance a second time, after the first one 
     * returned blocked throw an exception
	 */
    class TrajectoryBase : public Trajectory {
    public:
        /** @throw if a previous call to advance returned blocked
	     */
        AdvanceInfo advance() final;

        inline float getCumulatedCost() const final { return this->cumulatedCost.get(); };

    protected:
        TrajectoryBase() =  default;

        virtual AdvanceInfo advanceInternal() = 0;

        Cost cumulatedCost;

    private:
        AdvanceInfo lastAdvanceResult = AdvanceInfo::advanced;
    };
}

#endif