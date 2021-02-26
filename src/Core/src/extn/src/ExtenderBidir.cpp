/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ExtenderBidir.h>
#include <Error.h>

namespace mt {
	inline bool operator==(const BidirSolution& a, const BidirSolution& b) {
		return (std::get<0>(a) == std::get<0>(b)) && (std::get<1>(a) == std::get<1>(b));
	};

    ExtBidir::ExtBidir(const bool& cumulateSolutions, const double& deterministicCoefficient, Tree& leftTree, Tree& rightTree)
        : Extender<BidirSolution>(cumulateSolutions, deterministicCoefficient)
        , leftTree(leftTree)
        , rightTree(rightTree) {
    }

	class BidirSolutionFactory {
	public:
		BidirSolutionFactory(Problem& problem) 
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
		Problem& problem;
	};

    void ExtBidir::extend(const size_t& Iterations) {
		bool newSolFound = false;
		bool caso = true;
		Tree* Master = &this->leftTree;
		Tree* Slave = &this->rightTree;
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
			if (this->randEngine() < this->deterministicCoefficient) {
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
#ifdef SHOW_PROGRESS
			ProgressPrinter::show(this->iterationsDone);
#endif

			if (!this->cumulateSolutions && newSolFound) break;
		}
    }

	std::vector<NodeState> ExtBidir::computeSolutionSequence(const BidirSolution& sol) const {
		std::list<const NodeState*> states;
		const Node* cursor =std::get<0>(sol);
		while (nullptr != cursor) {
			states.push_front(&cursor->getState());
			cursor = cursor->getFather();
		}
		cursor = std::get<1>(sol);
		while (nullptr != cursor) {
			states.push_back(&cursor->getState());
			cursor = cursor->getFather();
		}

		return convert(states);
	}
}