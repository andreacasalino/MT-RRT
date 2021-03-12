/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_H
#define MT_RRT_EXTENDER_H

#include <TreeCore.h>
#include <sampler/engine/UniformEngine.h>
#include <list>
#include <set>

#ifdef SHOW_PROGRESS
#include <mutex>
#endif

namespace mt {
	template<typename Solution>
	class Extender {
	public:
		virtual ~Extender() = default;

		/** \brief Perform the specified number of estensions on a wrapped tree(s).
		\details This function may be called multiple times, for performing batch of extensions.
		If the cumulation of the solution was not enabled, calling this method when a solution was already
		found raise an exception.
		* @param[in] Iterations the number of extensions to perform
		*/
		virtual void extend(const size_t& Iterations) = 0;

		/** \brief Get the extensions so far done.
		*/
		inline std::size_t getIterationsDone() const { return this->iterationsDone; };

		inline const std::set<Solution>& getSolutions() const { return this->solutionsFound; };

		std::vector<NodeState> computeBestSolutionSequence() const {
			if (this->solutionsFound.empty()) {
				return {};
			}
			return this->computeSolutionSequence(*this->solutionsFound.begin());
		};

		template<typename ExtT>
		static std::vector<NodeState> computeBestSolutionSequence(const std::vector<ExtT> extenders) {
			std::set<Solution> solutions;
			for (auto itE = extenders.begin(); itE != extenders.end(); ++itE) {
				for (auto itSol = itE->solutionsFound.begin(); itSol != itE->solutionsFound.end(); ++itSol) {
					if (solutions.find(*itSol) == solutions.end()) {
						solutions.emplace(*itSol);
					}
				}
			}
			if (solutions.empty()) return {};
			return extenders.front().computeSolutionSequence(*solutions.begin());
		};

		inline bool isCumulating() const { return this->cumulateSolutions; };

		virtual std::vector<NodeState> computeSolutionSequence(const Solution& sol) const = 0;

	protected:
		Extender(const bool& cumulateSolutions, const double& deterministicCoefficient)
			: cumulateSolutions(cumulateSolutions)
			, deterministicCoefficient(deterministicCoefficient) {
		};

	// data
		sampling::UniformEngine randEngine;
		const bool				cumulateSolutions;
		const double			deterministicCoefficient;
		size_t					iterationsDone = 0;
		std::set<Solution>  	solutionsFound;
	};

	std::vector<NodeState> convert(const std::list<const NodeState*> nodes);

	TreeCore* convert(Tree* t);

	template<typename Extender>
	std::size_t getIterationsDone(const std::vector<Extender>& battery) {
		std::size_t iter = 0;
		for (auto it = battery.begin(); it != battery.end(); ++it) {
			iter += it->getIterationsDone();
		}
		return iter;
	};

#ifdef SHOW_PROGRESS
	class ProgressPrinter {
	public:
		static void show(const std::size_t& iter);

	private:
		static std::mutex coutMtx;
	};
#endif
}

#endif