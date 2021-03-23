/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_H
#define MT_RRT_TRAJECTORY_H

#include <Node.h>
#include <trajectory/Cost.h>

namespace mt::traj {
    /** @brief
     * blocked       -> when the advancement is not anymore possible, i.e. last state reached is not admitted by constraints
     * advanced      -> normal advancement. Last state reached is admitted by constraints.
     * targetReached -> when the last advancement led to the target state
	 */
    enum AdvanceInfo { blocked, advanced, targetReached };

    /** @brief Interface describing an optimal trajectory connecting 2 states, in a particular problem to solve.
     * Refer to METTERE. A cursor internally stored the state currently reached. When avancing this object,
     * the cursor is modified in order to traverse the trajectory.
	 */
    class Trajectory {
    public:
        virtual	~Trajectory() = default;

        Trajectory(const Trajectory&) = delete;
        Trajectory& operator=(const Trajectory&) = delete;

        /** @brief Move the internal cursor along the trajectory
	     */
        virtual AdvanceInfo advance() = 0;

        /** @return the current state of the cursor.
         * IMPORTANT: it is a no-sense value in case last advance() returned blocked
	     */
        virtual NodeState getCursor() const = 0;

        /** @return the cost to go from the beginning of the trajectory to the current cursor.
         * IMPORTANT: it is a no-sense value in case last advance() returned blocked
	     */
        virtual float getCumulatedCost() const = 0;

    protected:
        Trajectory() = default;
    };

    typedef std::unique_ptr<Trajectory> TrajectoryPtr;
}

#endif