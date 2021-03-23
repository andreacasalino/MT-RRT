/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_FACTORY_H
#define MT_RRT_TRAJECTORY_FACTORY_H

#include <Copiable.h>
#include <trajectory/Trajectory.h>

namespace mt::traj {
    /** @brief Creator of optimal trajectories, refer to METTERE.
     * Each specific problem to solve need to define and use its specific TrajectoryFactory
	 */
    class TrajectoryFactory : public Copiable<TrajectoryFactory>  {
    public:
        TrajectoryFactory(const TrajectoryFactory&) = delete;
        TrajectoryFactory& operator=(const TrajectoryFactory&) = delete;

    	/** @brief Evaluates the cost C(\tau), Section METTERE of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		 *  @param the starting node in the trajectory whose cost is to evaluate
		 *  @param the ending node in the trajectory whose cost is to evaluate
		 *  @param true when the constraints, see METTERE, need to be accounted. Cost::COST_MAX is in this case returned, when a feasible
         *  trajectory exists, but is not entirely contained in the admitted set METTERE  
         *  @return the cost to go of the trajectory connecting the states. Cost::COST_MAX is returned when a feasible trajectory does not exist.
		 */
        float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const;

        /** @param the starting state
         *  @param the ending state
         *  @return the optimal trajectory connecting the passed states. nullptr is returned in case a feasible trajectory does not exist
	     */
        virtual TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const = 0;

    protected:
        TrajectoryFactory() = default;

        virtual float cost2GoIgnoringConstraints(const NodeState& start, const NodeState& ending_node) const = 0;
    };

    typedef std::unique_ptr<TrajectoryFactory> TrajectoryFactoryPtr;
}

#endif