/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeMaster.h"

namespace mt::multiag {
    TreeSlave::TreeMaster::TreeMaster(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcrete(*problems.front(), std::move(root)) {
        this->slaves.reserve(problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            this->slaves.emplace_back(std::make_unique<TreeSlave>(*problems[k]));
        }
    }

    void TreeSlave::TreeMaster::gather() {
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Nodes& nodes = (*it)->getSlaveNodes();
            auto itN = nodes.begin();
            ++itN;
            for (itN; itN != nodes.end(); ++itN) {
                this->add(std::move(*itN));
            }
            nodes.clear();
        }
    }

    void TreeSlave::TreeMaster::dispatch() {
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Node* nearest = this->nearestNeighbour(this->problem.randomState(), this->getDelimiter());
            (*it)->getSlaveNodes().emplace_back( std::make_unique<Node>(nearest->getState()) );
        }
    }
}