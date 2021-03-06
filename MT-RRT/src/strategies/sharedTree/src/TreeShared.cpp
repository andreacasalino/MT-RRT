/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeShared.h"
#include <omp.h>

namespace mt::solver::shared {
    TreeShared::TreeShared(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeCore(std::move(root), *problems.front().get())
        , ProblemBattery(problems) {
    }

    Node* TreeShared::add(NodePtr node) {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->TreeCore::add(std::move(node));
    }

    Nodes::const_reverse_iterator TreeShared::rbegin() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->nodes.rbegin();
    };

    Problem* TreeShared::getProblem() const {
        return this->problems[omp_get_thread_num()];
    };
}