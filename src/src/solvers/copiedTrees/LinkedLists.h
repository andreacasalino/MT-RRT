/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LINKED_LISTS_H
#define MT_RRT_LINKED_LISTS_H

#include <list>
#include <vector>
#include <set>

namespace mt::copied {
    typedef int T;
    class LinkedLists {
    protected:
        LinkedLists() = default;

        static void link(std::vector<LinkedLists>& group) {
            std::size_t S = group.size() - 1;
            for (auto it = group.begin(); it != group.end(); ++it) {
                it->incomings.reserve(S);
                it->outgoings.reserve(S);
            }

            std::set<std::size_t> posTot;
            for (std::size_t c = 0; c < S + 1; ++c) {
                posTot.emplace(c);
            }

            for (std::size_t g = 0; g < group.size(); ++g) {
                std::set<std::size_t> pos = posTot;
                pos.erase(pos.find(g));
                for (auto itP = pos.begin(); itP != pos.end(); ++itP) {
                    group[g].incomings.emplace_back();
                    group[*itP].outgoings.emplace_back(&group[g].incomings.back());
                    ++itP;
                }
            }
        };

        std::vector<std::list<T>> incomings;
        std::vector<std::list<T>*> outgoings;
    };
}

#endif