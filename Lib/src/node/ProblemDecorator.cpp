/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/ProblemDecorator.h>
#include <Error.h>
using namespace std;

namespace mt::node {
	ProblemDecorator::ProblemDecorator(std::unique_ptr<Problem> wrapped) {
        if(nullptr == wrapped) throw Error("problem to wrap is nullptr");
        this->wrapped = std::move(wrapped);
	}
}