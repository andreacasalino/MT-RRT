/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeStarMaster.h"

namespace mt::multiag {
    TreeStarMaster::TreeStarMaster(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeMaster(problems, std::move(root)) {
    }

    void TreeStarMaster::gather() {
        std::vector<std::list<Rewird>> rews;
        rews.resize(this->slaves.size());
        for (auto it = this->slaves.begin(); it != this->slaves.end(); ++it) {
            Nodes& nodes = this->getSlaveNodes(**it);
            auto itN = nodes.begin();
            ++itN;
            for (itN; itN != nodes.end(); ++itN) {
                auto rew = this->computeRewirds(**itN, this->nodes.rbegin());
                for (auto it = rew.begin(); it != rew.end(); ++it) {
                    it->involved.setFather(&it->newFather, it->newCostFromFather);
                }
                this->add(std::move(*itN));
            }
            nodes.clear();
        }
    }
}