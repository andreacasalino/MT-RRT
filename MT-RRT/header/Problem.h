/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_PROBLEM_H
#define MT_RRT_PROBLEM_H

#include <Copiable.h>
#include <sampler/Sampler.h>
#include <trajectory/TrajectoryManager.h>
#include <Limited.h>

namespace mt {
	/** \brief Interface for the class describing the particular planning problem to solve. 
	\details It is crucial for addressing step A of the pipeline presented in Section 1.3 of the documentation.
	*/
	class Problem : public Copiable<Problem> {
	public:
		Problem& operator=(const Problem& ) = delete;

		inline std::unique_ptr<Problem> copy() const override { return std::unique_ptr<Problem>(new Problem(*this)); };

		/** \brief Performs a steering operation, Section 1.2.1 of the documentation, from a staring node to a target one.
		\details The node returned contains the steered state. In case a steering operation is not possible, a Node with a NULL State is returned.
		* @param[out] return the node with the steered configuration. 
		* @param[in] start the starting node from which the steer operation must be tried
		* @param[in] trg the target node to which the steer operation must be tried
		* @param[out] trg_reached returns true in case the steering was possible and led to reach the target node. Otherwise false is returned.
		*/
		// return nulltr if the steering was not possible
		NodePtr steer(Node& start, const NodeState& trg, bool& trg_reached);

		inline void setSteerTrials(const std::size_t& trials) { this->steerTrials.set(trials); };

		/** \brief Returns the cardinality of \mathcal{X}, Section 1.2.1 of the documentation, of the plannig problem handled by this object.
		*/
		inline std::size_t getProblemSize() const { return this->stateSpaceSize.get(); };
 
		/** \brief Returns the \gamma parameter, Section 1.2.3 of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		inline float getGamma() const { return this->gamma.get() * this->steerTrials.get(); };

		/** \brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		inline bool isProblemSimmetric() const { return this->simmetry; };

		inline sampling::Sampler* getSampler() const { return this->sampler.get(); };
		
		inline traj::TrajectoryManager* getTrajManager() const { return this->trajManager.get(); };

	protected:
		Problem(sampling::SamplerPtr sampler, traj::TrajectoryManagerPtr manager, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry = true);
		Problem(const Problem& o);

	// data
		const LowerLimited<std::size_t> stateSpaceSize; // lower bound 1
		const Positive<float> gamma;
		const bool simmetry;

		LowerLimited<std::size_t> steerTrials = LowerLimited<std::size_t>(1,1);

		sampling::SamplerPtr sampler;
		traj::TrajectoryManagerPtr trajManager;
	};

	typedef std::unique_ptr<Problem> ProblemPtr;
}

#endif