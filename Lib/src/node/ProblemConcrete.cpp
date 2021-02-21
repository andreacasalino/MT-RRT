/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/ProblemConcrete.h>
#include <Error.h>
using namespace std;

namespace mt::node {
	ProblemConcrete::ProblemConcrete(const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry)
		: stateSpaceSize(stateSpaceSize)
		, gamma(gamma)
		, simmetry(simmetry) {
		if (0 == this->stateSpaceSize) throw Error("invalid zero state space size");
		if (this->gamma < 0.f) throw Error("invalid negative gamma");
	}
}