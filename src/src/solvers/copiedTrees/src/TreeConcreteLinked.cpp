/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeConcreteLinked.h"
#include "../NodeLinked.h"

namespace mt::copied {
    TreeConcreteLinked::TreeConcreteLinked(Problem& problem, NodePtr root)
        : TreeConcrete(problem, std::move(root)) {
    }

    void TreeConcreteLinked::add(NodePtr node) {
        auto group = NodeLinked::make_copies(*node.get(), this->outgoings.size() + 1);
        this->TreeConcrete::add(std::move(node));
        for (std::size_t k = 0; k < this->outgoings.size(); ++k) {
            this->outgoings[k]->emplace_back(std::move(group[k]));
        }
    }

    void TreeConcreteLinked::gather() {
        for (auto it = this->incomings.begin(); it != this->incomings.end(); ++it) {
            for (auto itt = it->begin(); itt != it->end(); ++it) {
                this->TreeConcrete::add(std::move(*itt));
            }
            it->clear();
        }
    }
}
