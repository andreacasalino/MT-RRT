/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <Error.h>

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
		switch (mtStrategy) {
		case MTStrategy::Serial:
			this->lastSolution = std::move(this->solveSerial(start, end, rrtStrategy));
			break;
		case MTStrategy::MtQueryParall:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = std::move(this->solveQueryParall(start, end, rrtStrategy));
			break;
		case MTStrategy::MtSharedTree:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = std::move(this->solveSharedTree(start, end, rrtStrategy));
			break;
		case MTStrategy::MtCopiedTrees:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = std::move(this->solveCopiedTrees(start, end, rrtStrategy));
			break;
		case MTStrategy::MtMultiAgent:
			if (1 == this->problemcopies.size()) throw Error("can't use a multi thread strategy with just 1 thread");
			this->lastSolution = std::move(this->solveMultiAgent(start, end, rrtStrategy));
			break;
		default:
			throw Error("unrecognized strategy");
			break;
		}
	}

	void Solver::setThreadAvailability(const std::size_t& threads) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (0 == threads) {
			throw Error("number of threads should be at least 1");
		}
		if (threads <= this->problemcopies.size()) {
			this->problemcopies.resize(threads);
		}
		else {
			this->problemcopies.reserve(threads);
			while (this->problemcopies.size() < threads) {
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