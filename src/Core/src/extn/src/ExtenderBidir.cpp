/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/ExtenderBidir.h"
#include <Error.h>

namespace mt::solver::extn {
	inline bool operator<(const BidirSolution& a, const BidirSolution& b) {
		return (std::get<2>(a) < std::get<2>(b));
	};

	inline bool operator==(const BidirSolution& a, const BidirSolution& b) {
		return (std::get<0>(a) == std::get<0>(b)) && (std::get<1>(a) == std::get<1>(b));
	};

    Bidir::Bidir(const bool& cumulateSolutions, const float& deterministicCoefficient, tree::Tree& leftTree, tree::Tree& rightTree) 
        : Extender<BidirSolution>(cumulateSolutions, deterministicCoefficient)
        , leftTree(leftTree)
        , rightTree(rightTree) {
    }

	class BidirSolutionFactory {
	public:
		BidirSolutionFactory(problem::Problem& problem) 
			: problem(problem) {
		};

		BidirSolution makeSolution(const Node* a, const Node* b, const bool& caso) const {
			float cost = a->cost2Root() + this->problem.cost2Go(a->getState(), b->getState(), true) + b->cost2Root();
			if (caso) {
				return std::make_tuple(a, b, cost);
			}
			return std::make_tuple(b, a, cost);
		};

	private:
		problem::Problem& problem;
	};

    void Bidir::extend(const size_t& Iterations) {
		bool newSolFound = false;
		bool caso = true;
		tree::Tree* Master = &this->leftTree;
		tree::Tree* Slave = &this->rightTree;
		BidirSolutionFactory solFactory(this->leftTree.getProblem());

		auto add2Solutions = [this, &newSolFound, &solFactory](const Node* a, const Node* b, const bool& caso) {
			auto newSol = solFactory.makeSolution(a, b, caso);
			bool absent = true;
			for (auto it = this->solutionsFound.begin(); it != this->solutionsFound.end(); ++it) {
				if (*it == newSol) {
					absent = false;
					break;
				}
			}
			if (absent) {
				this->solutionsFound.emplace(newSol);
				newSolFound = true;
			}
		};

		auto extendSlave = [this, &newSolFound, &solFactory, &Slave, &Master, &caso, &add2Solutions](const Node* ext) {
			if (nullptr == ext) {
				auto temp = Slave->extendDeterministic(Master->getNodes().front()->getState());
				if (temp.second) {
					add2Solutions(temp.first, Master->getNodes().front().get(), !caso);
				}
			}
			else {
				auto temp = Slave->extendDeterministic(ext->getState());
				if (temp.second) {
					add2Solutions(temp.first, ext, !caso);
				}
			}
		};

		for (size_t k = 0; k < Iterations; k += 2) {
			if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) < this->deterministicCoefficient) {
				auto temp = Master->extendDeterministic(Slave->getNodes().front()->getState());
				if (temp.second) {
					add2Solutions(temp.first, Slave->getNodes().front().get(), caso);
				}
				else {
					extendSlave(temp.first);
				}
			}
			else {
				extendSlave(Master->extendRandom());
			}

			std::swap(Master, Slave);
			caso = !caso;

			++this->iterationsDone;
#ifdef _DISPLAY_ITERATIONS
			cout << "iteration " << this->Iterations_done << endl;
#endif // _DISPLAY_ITERATIONS

			if (!this->cumulateSolutions && newSolFound) break;
		}
    }
}