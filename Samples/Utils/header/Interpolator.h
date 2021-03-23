/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_INTERPOLATOR_H
#define MT_RRT_SAMPLE_INTERPOLATOR_H

#include <Problem.h>

namespace mt::sample {
    /** @brief Interpolates the solution found with a solver, using the optimal trajectories, METTERE, connecting
     * the intermediate states.
     *  @param solution foundby @Solver object
     *  @param the trajectory factory to use for computing the optimal trajectories
     *  @return the interpolated sequence of states representing the solution
	 */
    std::vector<NodeState> interpolate(const std::vector<NodeState>& solution, const traj::TrajectoryFactory& manager);
}

#endif