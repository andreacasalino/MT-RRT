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
#include <trajectory/TrajectoryFactory.h>
#include <Limited.h>

namespace mt {
	/** @brief Object storing the information needed to extend exploring trees.
	 */
	class Problem : public Copiable<Problem> {
	public:
	   /** @param the sampler to steal
		*  @param the trajectory factory to steal
		*  @param the dimension of the state space of the problem to solve. Refer to METTERE
		*  @param the parameters described in METTERE
		*  @param true when the problem is simmetric. Refer to METTERE
		*  @throw if sampler or manager are nullptr
	 	*/
		Problem(sampling::SamplerPtr sampler, traj::TrajectoryFactoryPtr manager, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry = true);
		Problem& operator=(const Problem& ) = delete;

	   /** @brief Used by @Solver: each working thread use its private Problem copy.
	 	*/
		inline std::unique_ptr<Problem> copy() const final { return std::unique_ptr<Problem>(new Problem(*this)); };

	   /** @brief Performs a steering operation, Section METTERE of the documentation, from a staring node to a target one.
		*  @param starting configuration
		*  @param target configuration to reach
		*  @param set true from the inside of this function, when the target was reached after steering
		*  @return the steered configuration. Is a nullptr when the steering was not possible at all
		*/
		NodePtr steer(Node& start, const NodeState& trg, bool& trg_reached);

	   /** @brief Sets the steering trials used when extending searching trees, refer to METTERE
		*/
		inline void setSteerTrials(const std::size_t& trials) { this->steerTrials.set(trials); };

	   /** @brief Returns the cardinality of \mathcal{X}, Section METTERE of the documentation, of the plannig problem handled by this object.
		*/
		inline std::size_t getProblemSize() const { return this->stateSpaceSize.get(); };
 
	   /** @brief Returns the \gamma parameter, Section METTERE of the documentation, regulating the near set size, that RRT* versions must compute.
		*/
		inline float getGamma() const { return this->gamma.get() * this->steerTrials.get(); };

	   /** @brief Returns true in case the planning problem handled by this object is symmetric, i.e. the cost to go from a node A to B is the same of the cost to go from B to A.
		*/
		inline bool isProblemSimmetric() const { return this->simmetry; };
	   
	   /** @return the stored sampler
		*/
		inline sampling::Sampler* getSampler() const { return this->sampler.get(); };

	   /** @return the stored trajectory factory
		*/
		inline traj::TrajectoryFactory* getTrajManager() const { return this->trajManager.get(); };

	protected:
		Problem(const Problem& o);

	// data
		const LowerLimited<std::size_t> stateSpaceSize; // lower bound 1
		const Positive<float> gamma;
		const bool simmetry;

		LowerLimited<std::size_t> steerTrials = LowerLimited<std::size_t>(1,1);

		sampling::SamplerPtr sampler;
		traj::TrajectoryFactoryPtr trajManager;
	};

	typedef std::unique_ptr<Problem> ProblemPtr;
}

#endif