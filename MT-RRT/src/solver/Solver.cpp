/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <solver/Solver.h>
#include <solver/Strategy.h>
#include <Error.h>
#include <omp.h>

namespace mt::solver {
	Solver::Solver(ProblemPtr problemDescription) {
		if (nullptr == problemDescription) {
			throw Error("problem description can't be null");
		}
		this->data = std::make_shared<SolverData>();
		this->data->problemsBattery.reserve(1);
		this->data->problemsBattery.emplace_back(std::move(problemDescription));
	}

	Solver::Solver(ProblemPtr problemDescription, std::unique_ptr<Strategy> solverStrategy)
		: Solver(std::move(problemDescription)) {
		this->setStrategy(std::move(solverStrategy));
	}

	void Solver::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);

		if(nullptr == this->strategy) {
			throw Error("solving strategy was not set");
		}
		if (start.size() != this->data->problemsBattery.front()->getProblemSize()) {
			throw Error("start configuration has inconsistent size");
		}
		if (end.size() != this->data->problemsBattery.front()->getProblemSize()) {
			throw Error("start configuration has inconsistent size");
		}

		if((RRTStrategy::Bidir == rrtStrategy) &&
		   (!this->data->problemsBattery.front()->isProblemSimmetric())) {
			throw Error("bidirectional strategy is not possible for this problem");
		}

		bool cumulFlagOld = this->strategy->getCumulateFlag();
		if (RRTStrategy::Star == rrtStrategy) {
			this->strategy->setCumulateFlag(true);
		}
		auto tic = std::chrono::high_resolution_clock::now();
		this->lastSolution = this->strategy->solve(start, end, rrtStrategy);
		this->lastSolution->time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tic);
		this->strategy->setCumulateFlag(cumulFlagOld);
		if(!this->data->saveComputedTrees) {
			this->lastSolution->trees.clear();
		}
	}

	void Solver::setThreadAvailability(const std::size_t& threads) {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		std::size_t th = threads;
		if (0 == th) {
#pragma omp parallel
			{
				th = static_cast<std::size_t>(omp_get_num_threads());
			}
		}
		if (th <= this->data->problemsBattery.size()) {
			this->data->problemsBattery.resize(th);
		}
		else {
			this->data->problemsBattery.reserve(th);
			while (this->data->problemsBattery.size() < th) {
				this->data->problemsBattery.emplace_back(this->data->problemsBattery.front()->copy());
			}
		}
	}

	void Solver::setStrategy(std::unique_ptr<Strategy> solverStrategy) {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		this->strategy = std::move(solverStrategy);
		this->strategy->shareSolverData(this->data);
	};

	std::unique_ptr<Strategy> Solver::extractStrategy() {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		if(nullptr != this->strategy) {
			this->strategy->forgetSolverData();
		}
		return std::move(this->strategy);
	};

	void Solver::setSteerTrials(const std::size_t& trials) {
		if (0 == trials) {
			throw Error("invalid steer trials");
		}
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		for (auto it = this->data->problemsBattery.begin(); it != this->data->problemsBattery.end(); ++it) {
			(*it)->setSteerTrials(trials);
		}
	}

	std::size_t Solver::getLastIterations() const {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		if (nullptr == this->lastSolution) return 0;
		return this->lastSolution->iterations;
	}

	std::vector<NodeState> Solver::copyLastSolution() const {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		if (nullptr == this->lastSolution) return {};
		return this->lastSolution->solution;
	}

	std::chrono::milliseconds Solver::getLastElapsedTime() const {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		return this->lastSolution->time;
	}

	std::size_t Solver::getThreadAvailability() const {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		return this->data->problemsBattery.size();
	}

	std::vector<TreePtrConst> Solver::extractLastTrees() {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		if (nullptr == this->lastSolution) return {};
		std::vector<TreePtrConst> temp;
		temp.reserve(this->lastSolution->trees.size());
		for (std::size_t t = 0; t < this->lastSolution->trees.size(); ++t) {
			temp.emplace_back(std::move(this->lastSolution->trees[t]));
		}
		this->lastSolution->trees.clear();
		return temp;
	}

	void Solver::saveTreesAfterSolve() {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		this->data->saveComputedTrees = true;
	}

	void Solver::discardTreesAfterSolve() {
		std::lock_guard<std::mutex> lock(this->data->solverMutex);
		this->data->saveComputedTrees = false;
	}
}