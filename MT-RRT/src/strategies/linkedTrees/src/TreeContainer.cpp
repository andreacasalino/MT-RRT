/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeContainer.h"
#include "../../Commons.h"
#include <omp.h>

namespace mt::solver::linked {
    template<typename TreeT>
    std::vector<std::unique_ptr<TreeLinked>> make_linked(Node& root, const std::vector<ProblemPtr>& problems) {
        checkBattery(problems);
        auto roots = NodeLinked::make_roots(root, problems.size());
        std::vector<std::unique_ptr<TreeLinked>> trees;
        trees.reserve(problems.size());
        for(std::size_t k=0; k<problems.size(); ++k) {
            trees.emplace_back( std::make_unique<TreeT>(std::move(roots[k]), *problems[k]) );
        }
        return trees;
    };  

    template<typename T>
    void link(const std::vector<std::unique_ptr<TreeLinked>>& contained) {
        std::vector<ListLinked<T>*> group;
        group.reserve(contained.size());
        for(std::size_t k=0; k<contained.size(); ++k) {
            group.emplace_back(dynamic_cast<ListLinked<T>*>(contained[k].get()));
        }
        ListLinked<T>::link(group);
    };

    TreeContainer::TreeContainer(NodePtr root, const std::vector<ProblemPtr>& problems) {
        this->contained = make_linked<TreeLinked>(*root, problems);
        link<NodePtr>(this->contained);
    }

    void TreeContainer::gather() {
        for (auto it = this->contained.begin(); it != this->contained.end(); ++it) {
            (*it)->gather();
        }
    }

    TreeStarContainer::TreeStarContainer(NodePtr root, const std::vector<ProblemPtr>& problems) {
        this->contained = make_linked<TreeStarLinked>(*root, problems);
        link<NodePtr>(this->contained);
        link<Rewire>(this->contained);
    }
}