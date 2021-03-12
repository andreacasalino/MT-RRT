/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TRAJECTORY_MANAGER_H
#define MT_RRT_TRAJECTORY_MANAGER_H

#include <Copiable.h>
#include <trajectory/Trajectory.h>

namespace mt::traj {
    class TrajectoryManager : public Copiable<TrajectoryManager>  {
    public:
    	/** \brief Evaluates the cost C(\tau), Section 1.2.3 of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		\details This cost doesn't account for constraints, but considers only the optimal unconstrained trajectory \tau leading from the starting to the ending node.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
        virtual float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const = 0;

        virtual TrajectoryPtr getTrajectory(const NodeState& start, const NodeState& ending_node) const = 0;
    };

    typedef std::unique_ptr<TrajectoryManager> TrajectoryManagerPtr;
}

#endif