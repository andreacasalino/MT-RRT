/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Tree.h>

namespace mt {
    Node* Tree::extendRandom() {
        auto temp = this->extend(this->getProblem().randomState());
        Node* pt = temp.first.get();
        this->add(std::move(temp.first));
        return pt;
    }
}