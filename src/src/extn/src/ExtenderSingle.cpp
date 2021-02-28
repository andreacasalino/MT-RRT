/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ExtenderSingle.h>
#include <random>

namespace mt {
    ExtSingle::ExtSingle(const bool& cumulateSolutions, const double& deterministicCoefficient, Tree& tree, const NodeState& target)
        : Extender<SingleSolution>(cumulateSolutions, deterministicCoefficient)
        , tree(tree)
        , target(target) {
    }

    void ExtSingle::extend(const size_t& Iterations) {
		bool newSolFound = false;
		for (size_t k = 0; k < Iterations; ++k) {
			if (this->randEngine() < this->deterministicCoefficient) {
				auto temp = this->tree.extend(this->target);
				if (nullptr != temp.first) {
					// check this solution was not already found
					bool absent = true;
					for (auto it = this->solutionsFound.begin(); it != this->solutionsFound.end(); ++it) {
						if (it->first == temp.first) {
							absent = false;
							break;
						}
					}
					if (absent) {
						this->solutionsFound.emplace(std::make_pair(temp.first, temp.first->cost2Root() + this->tree.getProblem().cost2Go(temp.first->getState(), this->target, true)));
						newSolFound = true;
					}
				}
				else this->tree.add(std::move(temp.second));
			}
			else  this->tree.extendRandom();

			++this->iterationsDone;
#ifdef SHOW_PROGRESS
			ProgressPrinter::show(this->iterationsDone);
#endif

			if (!this->cumulateSolutions && newSolFound) {
				break;
			}
		}
    }

	std::vector<NodeState> ExtSingle::computeSolutionSequence(const SingleSolution& sol) const {
		std::list<const NodeState*> states;
		const Node* cursor = sol.first;
		while (nullptr != cursor) {
			states.push_front(&cursor->getState());
			cursor = cursor->getFather();
		}
		states.push_back(&this->target);
		
		return convert(states);
	}
}