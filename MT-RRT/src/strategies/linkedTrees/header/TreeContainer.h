/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONTAINER_H
#define MT_RRT_TREE_CONTAINER_H

#include "../../ProblemBattery.h"
#include "../header/TreeStarLinked.h"
#include <omp.h>
#include <type_traits>

namespace mt::solver::linked {
    template<typename TreeT>
    class TreeContainer
        : public TreeT {
        static_assert(std::is_base_of<TreeLinked, TreeT>::value , "invalid tree type");
    public:
        TreeT& getTree() {
            std::size_t thId = omp_get_team_num();
            if(0 == thId) {
                return *this;
            }
            return *this->linkedTrees[thId - 1].get();
        };

        void doGather() {
            std::size_t thId = omp_get_team_num();
            if(0 == thId) {
                this->gather();
                return;
            }
            this->linkedTrees[thId - 1]->gather();
        };
        
        std::vector<TreeLinked*> getAsBattery() {
            std::vector<TreeLinked*> group;
            group.reserve(this->linkedTrees.size() + 1);
            group.emplace_back(this);
            for(auto it = this->linkedTrees.begin(); it!=this->linkedTrees.end(); ++it) {
                group.emplace_back(it->get());
            }
            return group;
        }

    protected:
        TreeContainer(NodePtr root, const std::vector<ProblemPtr>& problems)
            : TreeT(std::move(root), *problems.front()) {
            checkBattery(problems);
            this->linkedTrees = this->make_linked(problems);
        };

        std::vector<std::unique_ptr<TreeT>> make_linked(const std::vector<ProblemPtr>& problems) {
            auto roots = NodeLinked::make_roots(**this->rbegin(), problems.size());
            this->nodes.front() = std::move(roots.front());
            std::vector<std::unique_ptr<TreeT>> trees;
            trees.reserve(problems.size() - 1);
            for(std::size_t k=1; k<problems.size(); ++k) {
                trees.emplace_back(new TreeT(std::move(roots[k]), *problems[k]) );
            }
            return trees;
        };  

        template<typename T>
        void link() {
            std::vector<ListLinked<T>*> group;
            group.reserve(this->linkedTrees.size() + 1);
            group.emplace_back(this);
            for(auto it = this->linkedTrees.begin(); it!=this->linkedTrees.end(); ++it) {
                group.emplace_back(it->get());
            }
            ListLinked<T>::link(group);
        };

        std::vector<std::unique_ptr<TreeT>> linkedTrees;
    };

    class TreeLinkedContainer : public TreeContainer<TreeLinked> {
    public:
        TreeLinkedContainer(NodePtr root, const std::vector<ProblemPtr>& problems)
            : TreeContainer<TreeLinked>(std::move(root), problems) {
            this->link<NodePtr>();
        };
    };

    class TreeStarLinkedContainer : public TreeContainer<TreeStarLinked> {
    public:
        TreeStarLinkedContainer(NodePtr root, const std::vector<ProblemPtr>& problems)
            : TreeContainer<TreeStarLinked>(std::move(root), problems) {
            this->link<NodePtr>();
            this->link<Rewire>();
        };
    };
}

#endif