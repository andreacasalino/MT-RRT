/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_EXTENDER_H
#define MT_RRT_EXTENDER_H

#include <solver/Tree.h>
#include <set>

namespace mt::solver::extn {
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
		inline size_t getIterationsDone() { return this->iterationsDone; };

		inline const std::set<Solution>& getSolutions() { return this->solutionsFound; };

	protected:
		Extender(const bool& cumulateSolutions, const float& deterministicCoefficient) 
			: cumulateSolutions(cumulateSolutions)
			, deterministicCoefficient(deterministicCoefficient) {
		};

	// data
		const bool			cumulateSolutions;
		const float			deterministicCoefficient;
		size_t				iterationsDone = 0;
		std::set<Solution>  solutionsFound;
	};
}

#endif