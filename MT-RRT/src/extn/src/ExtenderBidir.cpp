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
        , leftTree(*convert(&leftTree))
        , rightTree(*convert(&rightTree)) {
		if (&this->leftTree == &this->rightTree) {
			throw Error("trees should be different for bidirectional extender");
		}
    }

	BidirSolution ExtBidir::makeSolution(const Node* a, const Node* b, const bool& caso) const {
		float cost = a->cost2Root() + this->leftTree.getProblem()->getTrajManager()->cost2Go(a->getState(), b->getState(), true) + b->cost2Root();
		if (caso) {
			return std::make_tuple(a, b, cost);
		}
		return std::make_tuple(b, a, cost);
	};

    void ExtBidir::extend(const size_t& Iterations) {
		bool newSolFound = false;
		bool caso = true;
		TreeCore* Master = &this->leftTree;
		TreeCore* Slave = &this->rightTree;

		auto add2Solutions = [this, &newSolFound](const Node* a, const Node* b, const bool& caso) {
			auto newSol = this->makeSolution(a, b, caso);
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

		auto extendSlave = [this, &newSolFound, &Slave, &Master, &caso, &add2Solutions](const Node* ext) {
			if (nullptr == ext) {
				auto temp = Slave->extend(Master->front()->getState());
				if (temp.second) {
					add2Solutions(temp.first->getFather(), Master->front(), !caso);
				}
				else Slave->add(std::move(temp.first));
			}
			else {
				auto temp = Slave->extend(ext->getState());
				if (temp.second) {
					add2Solutions(temp.first->getFather(), ext, !caso);
				}
				else Slave->add(std::move(temp.first));
			}
		};

		for (size_t k = 0; k < Iterations; k += 2) {
			if (this->randEngine() < this->deterministicCoefficient) {
				auto temp = Master->extend(Slave->front()->getState());
				if (temp.second) {
					add2Solutions(temp.first->getFather(), Slave->front(), caso);
				}
				else {
					Node* pt = temp.first.get();
					Master->add(std::move(temp.first));
					extendSlave(pt);
				}
			}
			else {
				extendSlave(Master->extendRandom());
			}

			std::swap(Master, Slave);
			caso = !caso;

			this->iterationsDone += 2;
#ifdef SHOW_PROGRESS
			ProgressPrinter::show(this->iterationsDone);
#endif

			if (!this->cumulateSolutions && newSolFound) {
				break;
			}
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

	std::vector<ExtBidir> make_battery(const bool& cumulateSolutions, const double& deterministicCoefficient, const std::vector<TreePtr>& treesA, const std::vector<TreePtr>& treesB) {
		if (treesA.size() != treesB.size()) {
			throw Error("inconsistent number of trees");
		}
		std::vector<ExtBidir> battery;
		battery.reserve(treesA.size());
		for (std::size_t k = 0; k < treesA.size(); ++k) {
			battery.emplace_back(cumulateSolutions, deterministicCoefficient, *treesA[k].get(), *treesB[k].get());
		}
		return battery;
	};
}