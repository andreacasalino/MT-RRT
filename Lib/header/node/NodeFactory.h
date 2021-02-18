/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_NODE_FACTORY_H
#define MT_RRT_NODE_FACTORY_H

#include <node/Node.h>
#include <node/Trajectory.h>
#include <list>
#include <memory>

namespace mt::node {
	/** \brief Interface for the class describing the particular planning problem to solve. 
	\details It is crucial for addressing step A of the pipeline presented in Section 1.3 of the documentation.
	*/
	class Node::NodeFactory {
	public:
		virtual	~NodeFactory() = default;

		NodeFactory(const NodeFactory& o) = delete;
		NodeFactory& operator=(const NodeFactory& o) = delete;

		/** \brief Evaluates the cost C(\tau), Section 1.2.3 of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		\details This cost doesn't account for constraints, but considers only the optimal unconstrained trajectory \tau leading from the starting to the ending node.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		inline float cost2Go(const Node& start, const Node& ending_node) { return this->computeTrajectory(start, ending_node)->cost2Go(); };

		/** \brief Evaluates the constrained cost of the trajectory going from the starting node to the ending one, for two nodes not already connected.
		\details This cost accounts for constraints. In case the constraints are violated along the nominal trajectory going from
		the starting node to the ending one, a FLT_MAX is returned. Otherwise, the cost returned is the one of the nominal trajectory, i.e. the
		one computed with I_Node_factory::Cost_to_go.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		float cost2GoConstraints(const Node& start, const  Node& trg);

		/** \brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section 1.2.1 of the documentation.
		\details This function is invoked for randomly growing a searching tree.
		* @param[out] return the random node computed . 
		*/
		Node createRandomNode();

		/** \brief Performs a steering operation, Section 1.2.1 of the documentation, from a staring node to a target one.
		\details The node returned contains the steered state. In case a steering operation is not possible, a Node with a NULL State is returned.
		* @param[out] return the node with the steered configuration. 
		* @param[in] start the starting node from which the steer operation must be tried
		* @param[in] trg the target node to which the steer operation must be tried
		* @param[out] trg_reached returns true in case the steering was possible and led to reach the target node. Otherwise false is returned.
		*/
		Node createSteered(const Node& start, const Node& trg, bool& trg_reached);

		/** \brief Generates a node with the same state and father of the node passed as input.
        * The same father is assumed.
		* @param[out] return the cloned node. 
		* @param[in] o the node to clone
		*/
		Node createCloned(const Node& o) const;

		/** \brief Builds a new root for a tree.
		\details The root is a node having a NULL father.
		* @param[in] state the state that will be contained in the root to create.
		* @param[out] return the created root . 
		*/
		Node createRoot(const std::vector<float>& state) const;

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		inline std::size_t getStateSize() const { return this->State_size; };

		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		inline float getGamma() const { return this->Gamma_coeff * this->Steer_max_iteration; };

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		inline bool isProblemSimmetric() const { return this->Traj_symmetric; };

		/** \brief Set the maximal number of iterations that are considered when doing a steering operation, Figure 2.3 of the documentation (even if the concept is applied to any kind of problem).
		\details By default this value is assumed equal to 1: a single point alogn the nominal trajectory is verified to be in the admitted region and possibly assumed as steered configuration.
		When a greater value is set, multiple steering operations are tried, in order to get as much as possible close to the target configuration.
		* @param[in] Iter_number the number of iterations to consider when doing the steering process. Pass 0 to set this value to infinity: the steering is tried till reaching the target or a configuration 
		outside of the admitted region.
		*/
		inline void setSteerTrials(const size_t& Iter_number) { this->Steer_max_iteration = Iter_number; };

		/** \brief Set the maximal number of iterations that are considered when checking the connectivity of two states, see also I_Node_factory::Cost_to_go_constraints.
		\details By default this value is assumed equal to 0: the trajectory connecting the nodes is traversed till reaching the end or finding an unfeasible state without saturations.
		* @param[in] Iter_number the number of iterations to consider
		outside of the admitted region.
		*/
		inline void setcost2GoConstraintsTrials(const size_t& Iter_number) { this->Cost_to_go_constraints_max_iterations = Iter_number; };

		/** \brief Takes a series of waypoints and interpolate it, adding some intermediate states along the optimal trajectories connecting the waypoints.
		\details The additonal states are inserted in the passed list in the proper position.
		* @param[in] waypoints_to_interpolate the chain of waypoints to interpolate
		* @param[out] cost_total the total cost to go for going from traversing all the waypoints in the chain. Computed only if you pass a value different from nullptr
		*/
		float interpolate(std::list<std::vector<float>>& waypoints_to_interpolate);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// To customize for the specific planning problem to solve
//
		/** \brief used for cloning this object: a deep copy must be implemented.
		\details This function is invoked by parallel planners for dispatching copies of this class to the other working threads.
		In this way, the threads must not be forced to synchronize for accessing the methods of an I_Node_factory.
		Therefore, when deriving your own factory describing your own problem, be carefull to avoid shallow copies and implement deep copies.
		* @param[out] return a clone of this object
		*/
		virtual std::unique_ptr<NodeFactory> copy() = 0;

	protected:
		/** \brief internally called by I_Node_factory::Random_node().
		\details The passed random_state is an array of values already with the cardinality of \mathcal{X}, which must be
		set to random values by this function. Use I_Node_factory::rand to internally draw samples, since rand() is not thread safe.
		*/
		virtual void createRandomState(std::vector<float>& random_state) = 0;

        virtual std::unique_ptr<Trajectory> computeTrajectory(const Node& start, const Node& end) = 0;
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		

		/** \brief Constructor of a new problem
		* @param[in] X_size the cardinality of the space where the planning problem lives
		* @param[in] gamma the \gamma (see Node::I_Node_factory::Get_Gamma()) parameter used by RRT*
		* @param[in] traj_symm_flag a flag explaining whether the problem is symmetric or not, see Node::I_Node_factory::Get_symm_flag()
		*/
		NodeFactory(const size_t& X_size, const float& gamma, const bool& traj_symm_flag);

	private:
	// data
		const size_t								StateSpaceSize;
		const bool									Traj_symmetric;
		float										Gamma_coeff;
		size_t										Steer_max_iteration = 1;
		size_t										Cost_to_go_constraints_max_iterations = 0;

		unsigned int								random_engine_state = 0;
	};
}

#endif