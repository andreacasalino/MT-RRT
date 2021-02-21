/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_PROBLEM_CONCRETE_H
#define MT_RRT_PROBLEM_CONCRETE_H

#include <node/Problem.h>

namespace mt::node {
	class ProblemConcrete : public Problem {
	public:
		inline std::size_t getProblemSize() const override { return this->stateSpaceSize; };

		inline float getGamma() const override { return this->gamma; };

		inline bool isProblemSimmetric() const override { return this->simmetry; };

	protected:
		ProblemConcrete(const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry);

	private:
		// data
		const std::size_t stateSpaceSize;
		const float gamma;
		const bool simmetry;
	};
}

#endif