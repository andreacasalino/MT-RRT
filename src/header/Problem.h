/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_PROBLEM_H
#define MT_RRT_PROBLEM_H

#include <sampler/Sampler.h>
#include <trajectory/Manager.h>

namespace mt {
	/** \brief Interface for the class describing the particular planning problem to solve. 
	\details It is crucial for addressing step A of the pipeline presented in Section 1.3 of the documentation.
	*/
	class Problem {
	public:
		virtual	~Problem() = default;
		Problem& operator=(const Problem& ) = delete;

		Problem(Problem&&) = delete;
		Problem& operator=(Problem&&) = delete;

		virtual std::unique_ptr<Problem> copy() const = 0;

		/** \brief Evaluates the cost C(\tau), Section 1.2.3 of the documentation, of the trajectory \tau going from the starting node to the ending one, for two nodes not already connected.
		\details This cost doesn't account for constraints, but considers only the optimal unconstrained trajectory \tau leading from the starting to the ending node.
		* @param[out] result the computed cost
		* @param[in] start the starting node in the trajectory whose cost is to evaluate
		* @param[in] ending_node the ending node in the trajectory whose cost is to evaluate
		*/
		inline float cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const { return this->trajManager->cost2Go(start, ending_node, ignoreConstraints); };

		/** \brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section 1.2.1 of the documentation.
		\details This function is invoked for randomly growing a searching tree.
		* @param[out] return the random node computed . 
		*/
		inline NodeState randomState() { return this->sampler->randomState(); };

		/** \brief Performs a steering operation, Section 1.2.1 of the documentation, from a staring node to a target one.
		\details The node returned contains the steered state. In case a steering operation is not possible, a Node with a NULL State is returned.
		* @param[out] return the node with the steered configuration. 
		* @param[in] start the starting node from which the steer operation must be tried
		* @param[in] trg the target node to which the steer operation must be tried
		* @param[out] trg_reached returns true in case the steering was possible and led to reach the target node. Otherwise false is returned.
		*/

		// return nulltr if the steering was not possible
		NodePtr steer(Node& start, const NodeState& trg, bool& trg_reached);

		void setSteerTrials(const std::size_t& trials);

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		inline std::size_t getProblemSize() const { return this->stateSpaceSize; };
 
		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		inline float getGamma() const { return this->gamma * this->steerTrials; };

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		inline bool isProblemSimmetric() const { return this->simmetry; };

	protected:
		Problem(sampling::SamplerPtr sampler, traj::ManagerPtr manager, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry = true);
		Problem(const Problem& o);

	// data
		const std::size_t stateSpaceSize;
		const float gamma;
		const bool simmetry;
		
		std::size_t steerTrials = 1;

		sampling::SamplerPtr sampler;
		traj::ManagerPtr trajManager;
	};

	typedef std::unique_ptr<Problem> ProblemPtr;
}

#endif