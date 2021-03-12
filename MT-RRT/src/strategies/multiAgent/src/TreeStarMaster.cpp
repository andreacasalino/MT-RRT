/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStarMaster.h"

namespace mt::solver::multiag {
    TreeStarMaster::TreeStarMaster(NodePtr root, const std::vector<ProblemPtr>& problems)
        : TreeMaster(std::move(root), problems) {
    }

    void TreeStarMaster::gather() {
        std::vector<std::list<Rewire>> rews;
        rews.resize(this->slaves.size());
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Nodes& nodes = (*it)->getNodes();
            auto itN = nodes.begin();
            ++itN;
            for (itN; itN != nodes.end(); ++itN) {
                auto rew = this->computeRewires(**itN);
                for (auto it = rew.begin(); it != rew.end(); ++it) {
                    it->involved.setFather(&it->newFather, it->newCostFromFather);
                }
                this->add(std::move(*itN));
            }
            nodes.clear();
        }
    }
}