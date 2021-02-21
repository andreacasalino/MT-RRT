/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_PROBLEM_H
#define MT_RRT_PROBLEM_H

#include <node/Node.h>
#include <memory>

namespace mt::node {
	/** \brief Interface for the class describing the particular planning problem to solve. 
	\details It is crucial for addressing step A of the pipeline presented in Section 1.3 of the documentation.
	*/
	class Problem {
	public:
		virtual	~Problem() = default;

		Problem(const Problem& o) = delete;
		Problem& operator=(const Problem& o) = delete;

		/** \brief Evaluates the cost C(\tau), Section 1.2.3 of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		\details This cost doesn't account for constraints, but considers only the optimal unconstrained trajectory \tau leading from the starting to the ending node.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		virtual float cost2Go(const Node& start, const Node& ending_node, const bool& ignoreConstraints = true) = 0;

		/** \brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section 1.2.1 of the documentation.
		\details This function is invoked for randomly growing a searching tree.
		* @param[out] return the random node computed . 
		*/
		virtual NodeState randomState() = 0;

		/** \brief Performs a steering operation, Section 1.2.1 of the documentation, from a staring node to a target one.
		\details The node returned contains the steered state. In case a steering operation is not possible, a Node with a NULL State is returned.
		* @param[out] return the node with the steered configuration. 
		* @param[in] start the starting node from which the steer operation must be tried
		* @param[in] trg the target node to which the steer operation must be tried
		* @param[out] trg_reached returns true in case the steering was possible and led to reach the target node. Otherwise false is returned.
		*/
		virtual NodeState steeredState(const Node& start, const Node& trg, bool& trg_reached) = 0;

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		virtual std::size_t getProblemSize() const = 0;

		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		virtual float getGamma() const = 0;

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		virtual bool isProblemSimmetric() const = 0;

		/** \brief used for cloning this object: a deep copy must be implemented.
		\details This function is invoked by parallel planners for dispatching copies of this class to the other working threads.
		In this way, the threads must not be forced to synchronize for accessing the methods of an I_Node_factory.
		Therefore, when deriving your own factory describing your own problem, be carefull to avoid shallow copies and implement deep copies.
		* @param[out] return a clone of this object
		*/
		virtual std::unique_ptr<Problem> copy() = 0;
	};
}

#endif