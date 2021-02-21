/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_PROBLEM_DECORATOR_H
#define MT_RRT_PROBLEM_DECORATOR_H

#include <node/Problem.h>

namespace mt::node {
	class ProblemDecorator : public Problem {
	public:
		inline float cost2Go(const Node& start, const Node& ending_node, const bool& ignoreConstraints) override { return this->wrapped->cost2Go(start, ending_node, ignoreConstraints); };

		inline NodeState randomState() override { return this->wrapped->randomState(); };

		inline NodeState steeredState(const Node& start, const Node& trg, bool& trg_reached) override { return this->wrapped->steeredState(start, trg, trg_reached); };

		inline std::size_t getProblemSize() const override { return this->wrapped->getProblemSize(); };

		inline float getGamma() const override { return this->wrapped->getGamma(); };

		inline bool isProblemSimmetric() const override { return this->wrapped->isProblemSimmetric(); };

	protected:
		ProblemDecorator(std::unique_ptr<Problem> wrapped);

		std::unique_ptr<Problem> wrapped;
	};
}

#endif