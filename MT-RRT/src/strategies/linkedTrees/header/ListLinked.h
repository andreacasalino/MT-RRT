/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LIST_LINKED_H
#define MT_RRT_LIST_LINKED_H

#include <list>
#include <set>
#include <memory>

namespace mt::solver::linked {
    template<typename T>
    class ListLinked {
    public:
        ListLinked(const ListLinked<T>&) = delete;
        ListLinked& operator=(const ListLinked<T>&) = delete;

        static void link(std::vector<ListLinked<T>*>& group) {
            for (auto it = group.begin(); it != group.end(); ++it) {
                (*it)->incomings.clear();
                (*it)->outgoings.clear();
            }

            std::set<std::size_t> posTot;
            for (std::size_t c = 0; c < group.size(); ++c) {
                posTot.emplace(c);
            }

            for (std::size_t g = 0; g < group.size(); ++g) {
                std::set<std::size_t> pos = posTot;
                pos.erase(pos.find(g));
                for (auto itP = pos.begin(); itP != pos.end(); ++itP) {
                    group[g]->incomings.emplace_back();
                    group[*itP]->outgoings.push_back(&group[g]->incomings.back());
                }
            }
        };

    protected:
        ListLinked() = default;

        template<typename Action>
        void gatherResult(const Action& action) {
            for (auto in = this->incomings.begin(); in != this->incomings.end(); ++in) {
                for (auto it = in->begin(); it != in->end(); ++it) {
                    action(*it);
                }
                in->clear();
            }
        };
        
        std::list<std::list<T>> incomings;
        std::list<std::list<T>*> outgoings;
    };
}

#endif
