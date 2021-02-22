/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <solver/Solver.h>
#include <Error.h>

namespace mt::solver {
	Solver::Solver(ProblemPtr problemDescription) {
		if (nullptr == problemDescription) {
			throw Error("problem description can't be nullptr");
		}
		this->problemcopies.reserve(1);
		this->problemcopies.emplace_back(std::move(problemDescription));
	}

	void Solver::RRTSingle(const NodeState& start, const NodeState& end, const Strategy& strategy) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->lastSolution.reset(new SolutionInfo());
		// todo
	}

	void Solver::RRTConnect(const NodeState& start, const NodeState& end, const Strategy& strategy) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->lastSolution.reset(new SolutionInfo());
		// todo
	}

	void Solver::RRTStar(const NodeState& start, const NodeState& end, const Strategy& strategy) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->lastSolution.reset(new SolutionInfo());
		bool originalCulVal = this->Cumulate_sol;
		this->Cumulate_sol = true;
		// todo
		this->Cumulate_sol = originalCulVal;
	}

	void Solver::setThreadAvailability(const std::size_t& threads) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (0 == threads) {
			throw Error("number of threads should be at least 1");
		}
		if (this->problemcopies.capacity() < threads) {
			this->problemcopies.reserve(threads);
			while (threads != this->problemcopies.size()) {
				this->problemcopies.emplace_back(this->problemcopies.front()->copy());
			}
		}
		this->problemcopies.resize(threads);
	}

	void Solver::setDeterminism(const double& coeff) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (coeff < 0.01f) {
			throw Error("deterministic coefficient is too low");
		}
		if (coeff > 0.99f) {
			throw Error("deterministic coefficient is too high");
		}
		this->Deterministic_coefficient = coeff;
	}

	void Solver::setMaxIterations(const std::size_t& iter) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (iter < 10) {
			throw Error("max iterations is too low");
		}
		this->Iterations_Max = iter;
	}

	void Solver::setCumulateOption(const bool& val) {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		this->Cumulate_sol = true;
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

	std::vector<TreePtr> Solver::getLastTrees() {
		std::lock_guard<std::mutex> lock(this->dataMtx);
		if (nullptr == this->lastSolution) return {};
		std::vector<TreePtr> temp = std::move(this->lastSolution->trees);
		this->lastSolution->trees.clear();
		return temp;
	}
}