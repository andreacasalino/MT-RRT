/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/ExtenderSingle.h"
#include <Error.h>

namespace mt::solver::extn {
    inline bool operator<(const SingleSolution& a, const SingleSolution& b) {
        return (a.second < b.second);
    };

    Single::Single(const bool& cumulateSolutions, const float& deterministicCoefficient, tree::Tree& tree, const NodeState& target)
        : Extender<SingleSolution>(cumulateSolutions, deterministicCoefficient)
        , tree(tree)
        , target(target) {
    }

    void Single::extend(const size_t& Iterations) {
		for (size_t k = 0; k < Iterations; ++k) {
			if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < this->deterministicCoefficient) {
				auto temp = this->tree.extendDeterministic(this->target);
				throw std::runtime_error("is it correct to add the node in extend even if this solution was already found");
				if (temp.second) {
					// check this solution was not already found
					bool absent = true;
					for (auto it = this->solutionsFound.begin(); it != this->solutionsFound.end(); ++it) {
						if (it->first == temp.first) {
							absent = false;
							break;
						}
					}
					if (absent) {
						this->solutionsFound.emplace({temp.first, temp.first->cost2Root() + this->tree.getProblem().cost2Go(temp.first->getState(), this->target, true)});
					}
				}
			}
			else  this->tree.extendRandom();

			++this->iterationsDone;
#ifdef DISPLAY_ITERATIONS
			cout << "iteration " << this->Iterations_done << endl;
#endif // _DISPLAY_ITERATIONS
		}
    }
}