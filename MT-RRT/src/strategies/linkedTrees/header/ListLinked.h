/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_LIST_LINKED_H
#define MT_RRT_LIST_LINKED_H

#include <vector>
#include <list>
#include <set>
#include <map>
#include <memory>

namespace mt::solver::linked {
    template<typename T>
    class ListLinked {
    public:
        ListLinked(const ListLinked<T>&) = delete;
        ListLinked& operator=(const ListLinked<T>&) = delete;

        static void link(std::vector<ListLinked<T>*>& group) {
            std::set<std::size_t> posTot;
            std::size_t c;
            for (c = 0; c < group.size(); ++c) {
                posTot.emplace(c);
            }
            // generate buffers
            std::vector< std::map<std::size_t , shared_buffer> > buff;
            buff.resize(group.size());
            for(c=0; c<group.size(); ++c) {
                auto index = posTot;
                index.erase(index.find(c));
                for(auto it = index.begin(); it!=index.end(); ++it) {
                    buff[c].emplace(*it; std::make_shared<std::list<T>>());
                }
            }
            // fill incomings
            for (c = 0; c < group.size(); ++c) {
                group[c]->incomings.clear();
                group[c]->incomings.reserve(buff[c].size());
                for(auto it = buff[c].begin(); it!=buff[c].end(); ++it) {
                    group[c]->incomings.emplace_back(it->second);
                }
            }
            // fill outgoings
            for (c = 0; c < group.size(); ++c) {
                auto index = posTot;
                index.erase(index.find(k));
                group[c]->outgoings.clear();
                group[c]->outgoings.reserve(index.size());
                for(std::size_t i=index.begin(); i!=index.end(); ++i) {
                    group[c]->outgoings.emplace_back(buff[*i].find(c)->second);
                }
            }
        };

    protected:
        ListLinked() = default;

        template<typename Action>
        void gatherResult(const Action& action) {
            for (auto in = this->incomings.begin(); in != this->incomings.end(); ++in) {
                for (auto it = (*in)->begin(); it != (*in)->end(); ++it) {
                    action(*it);
                }
                in->clear();
            }
        };
        
        typedef std::shared_ptr<std::list<T>> shared_buffer;
        std::vector<shared_buffer> incomings;
        std::vector<shared_buffer> outgoings;
    };
}

#endif
