/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeSlave.h"

namespace mt::solver::multiag {
    TreeSlave::TreeSlave(Problem& problem)
        : TreeCore(std::make_unique<Node>(NodeState{0.f}), problem) {
        this->nodes.clear();
    }
}