/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <solver/Strategy.h>
#include <Error.h>

namespace mt::solver {
	Strategy::Strategy()
		: Deterministic_coefficient(0.01, 0.99, 0.2)
		, Iterations_Max(10, 100) {
	}
}