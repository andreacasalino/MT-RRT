/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeMaster.h"
#include "../../ProblemBattery.h"
#include <omp.h>

namespace mt::solver::multiag {
    TreeMaster::TreeMaster(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeCore(std::move(root), *problems.front()) {
        checkBattery(problems);
        this->slaves.reserve(problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            this->slaves.emplace_back(std::make_unique<TreeSlave>(*problems[k]));
        }
    }

    void TreeMaster::gather() {
        if(0 != omp_get_thread_num()) {
            return;
        }
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Nodes* nodes = (*it)->getNodes();
            for (auto itN = nodes->begin(); itN != nodes->end(); ++itN) {
                this->add(std::move(*itN));
            }
            nodes->clear();
        }
    }

    void TreeMaster::dispatch() {
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Node* nearest = this->nearestNeighbour(this->getProblem()->getSampler()->randomState());
            (*it)->getNodes()->clear();
            (*it)->getNodes()->emplace_back( std::make_unique<Node>(nearest->getState()) );
            (*it)->getNodes()->back()->setFather(nearest, 0.f);
        }
    }
}