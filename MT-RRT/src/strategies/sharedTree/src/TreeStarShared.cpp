/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStarShared.h"

namespace mt::solver::shared {
    TreeStarShared::TreeStarShared(NodePtr root, const std::vector<ProblemPtr>& problems) 
        : TreeStar<TreeShared>(std::move(root), problems) {
    }

    Node* TreeStarShared::add(NodePtr node) {
        Node* temp = this->TreeShared::add(std::move(node));
        if(nullptr != nullptr) {
            auto rew = this->TreeRewirer::computeRewires(*temp);
            std::lock_guard<std::mutex> lock(this->mtx);
            for (auto it = rew.begin(); it != rew.end(); ++it) {
                it->involved.setFather(&it->newFather, it->newCostFromFather);
            }
        }
        return temp;
    }
}