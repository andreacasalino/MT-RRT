/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <Error.h>
#include <omp.h>

namespace mt {
	Solver::Solver(ProblemPtr problemDescription) {
		if (nullptr == problemDescription) {
			throw Error("problem description can't be null");
		}
		this->problemcopies.reserve(1);
		this->problemcopies.emplace_back(std::move(problemDescription));
	}

	void Solver::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy, const MTStrategy& mtStrategy) {
		std::lock_guard<std::mutex> lock(this->dataMtx);

		if (start.size() != this->problemcopies.front()->getProblemSize()) {
			throw Error("start configuration has inconsistent size");
		}
		if (end.size() != this->problemcopies.front()->getProblemSize()) {
			throw Error("start configuration has inconsistent size");
		}

		bool cumulFlagOld = this->parameters.Cumulate_sol;
		if (RRTStrategy::Star == rrtStrategy) {
			this->parameters.Cumulate_sol = true;
		}
		auto tic = std::chrono::high_resolution_clock::now();
		switch (mtStrategy) {
		case MTStrategy::Serial:
			this->lastSolution = this->solveSerial(start, end, rrtStrategy);
			break;
		case MTStrategy::MtQueryParall:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = this->solveQueryParall(start, end, rrtStrategy);
			break;
		case MTStrategy::MtSharedTree:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = this->solveSharedTree(start, end, rrtStrategy);
			break;
		case MTStrategy::MtCopiedTrees:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = this->solveCopiedTrees(start, end, rrtStrategy);
			break;
		case MTStrategy::MtMultiAgent:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = this->solveMultiAgent(start, end, rrtStrategy);
			break;
		default:
			throw Error("unrecognized strategy");
			break;
		}
		this->lastSolution->time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - tic);
		this->parameters.Cumulate_sol = cumulFlagOld;
	}

	void Solver::setThreadAvailability(const std::size_t& threads) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		std::size_t th = threads;
		if (0 == th) {
#pragma omp parallel
			{
				th = static_cast<std::size_t>(omp_get_num_threads());
			}
		}
		if (th <= this->problemcopies.size()) {
			this->problemcopies.resize(th);
		}
		else {
			this->problemcopies.reserve(th);
			while (this->problemcopies.size() < th) {
				this->problemcopies.emplace_back(this->problemcopies.front()->copy());
			}
		}
	}

	void Solver::setDeterminism(const double& coeff) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (coeff < 0.01f) {
			throw Error("deterministic coefficient is too low");
		}
		if (coeff > 0.99f) {
			throw Error("deterministic coefficient is too high");
		}
		this->parameters.Deterministic_coefficient = coeff;
	}

	void Solver::setMaxIterations(const std::size_t& iter) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (iter < 10) {
			throw Error("max iterations is too low");
		}
		this->parameters.Iterations_Max = iter;
	}

	void Solver::setCumulateOption(const bool& val) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->parameters.Cumulate_sol = val;
	}

	void Solver::setSteerTrials(const std::size_t& trials) {
		if (0 == trials) {
			throw Error("invalid steer trials");
		}
		std::lock_guard<std::mutex> lock(this->dataMtx);
		for (auto it = this->problemcopies.begin(); it != this->problemcopies.end(); ++it) {
			(*it)->setSteerTrials(trials);
		}
	}

	void Solver::setReallignmentCoeff(const double& reallCoeff) {
		if (reallCoeff < 0.001) {
			throw Error("reallignment coefficient is too low");
		}
		if (reallCoeff > 1) {
			throw Error("reallignment coefficient is too high");
		}
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->parameters.reallignment_coeff = reallCoeff;
	}

	std::size_t Solver::getLastIterations() const {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (nullptr == this->lastSolution) return 0;
		return this->lastSolution->iterations;
	}

	std::vector<NodeState> Solver::getLastSolution() const {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (nullptr == this->lastSolution) return {};
		return this->lastSolution->solution;
	}

	std::chrono::milliseconds Solver::getLastElapsedTime() const {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		return this->lastSolution->time;
	}

	std::size_t Solver::getThreadAvailability() const {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		return this->problemcopies.size();
	}

	std::vector<TreePtrConst> Solver::getLastTrees() {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (nullptr == this->lastSolution) return {};
		std::vector<TreePtrConst> temp;
		temp.reserve(this->lastSolution->trees.size());
		for (std::size_t t = 0; t < this->lastSolution->trees.size(); ++t) {
			temp.emplace_back(std::move(this->lastSolution->trees[t]));
		}
		this->lastSolution->trees.clear();
		return temp;
	}
}