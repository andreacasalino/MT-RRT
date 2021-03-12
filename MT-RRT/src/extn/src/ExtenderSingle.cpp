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
				if (temp.second) {
					// check this solution was not already found
					bool absent = true;
					for (auto it = this->solutionsFound.begin(); it != this->solutionsFound.end(); ++it) {
						if (it->first == temp.first->getFather()) {
							absent = false;
							break;
						}
					}
					if (absent) {
						this->solutionsFound.emplace(std::make_pair(temp.first->getFather(), temp.first->cost2Root() ));
						newSolFound = true;
					}
				}
				else this->tree.add(std::move(temp.first));
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

	std::vector<ExtSingle> make_battery(const bool& cumulateSolutions, const double& deterministicCoefficient, const std::vector<TreePtr>& trees, const NodeState& target) {
		std::vector<ExtSingle> battery;
		battery.reserve(trees.size());
		for (auto it = trees.begin(); it != trees.end(); ++it) {
			battery.emplace_back(cumulateSolutions, deterministicCoefficient, *it->get(), target);
		}
		return battery;
	};
}