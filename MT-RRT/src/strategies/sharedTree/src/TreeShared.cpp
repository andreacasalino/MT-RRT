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
        , problems(make_battery(problems)) {
    }

    Node* TreeShared::add(NodePtr node) {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->TreeCore::add(std::move(node));
    }

    Nodes::const_reverse_iterator TreeShared::rend() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->nodes.rend();
    };

    Nodes::const_reverse_iterator TreeShared::rbegin() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->nodes.rbegin();
    };

    Problem& TreeShared::getProblem() {
        return *this->problems[omp_get_thread_num()];
    };

    const Problem& TreeShared::getProblemConst() const {
        return *this->problems[omp_get_thread_num()];
    };
}