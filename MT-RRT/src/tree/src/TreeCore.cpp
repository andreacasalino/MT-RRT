/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeCore.h>
#include <Error.h>

namespace mt {
    TreeCore::TreeCore(NodePtr root, Problem& problem)
        : TreeIterable(std::move(root)) {
        this->problem = &problem;
    }

    Node* TreeCore::extendRandom() {
        auto temp = this->extend(this->getProblem()->getSampler()->randomState());
        return this->add(std::move(temp.first));
    }
}