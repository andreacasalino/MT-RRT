/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeConcreteCritical.h"
#include <omp.h>

namespace mt::shared {
    TreeConcreteCritical::TreeConcreteCritical(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcrete(*problems.front(), std::move(root))
        , problems(make_battery(problems)) {
    }

    void TreeConcreteCritical::add(NodePtr node) {
        std::lock_guard<std::mutex> lock(this->mtx);
        this->TreeConcrete::add(std::move(node));
    }

    const Nodes& TreeConcreteCritical::getNodes() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->TreeConcrete::getNodes();
    };

    Problem& TreeConcreteCritical::getProblem() {
        return *this->problems[omp_get_thread_num()];
    };

    const Problem& TreeConcreteCritical::getProblem() const {
        return *this->problems[omp_get_thread_num()];
    };

    Nodes::const_reverse_iterator TreeConcreteCritical::getDelimiter() const {
        std::lock_guard<std::mutex> lock(this->mtx);
        return this->TreeConcrete::getDelimiter();
    };
}