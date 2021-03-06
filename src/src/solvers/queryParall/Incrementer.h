/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_INCREMENTER_H
#define MT_RRT_INCREMENTER_H

#include <Tree.h>

namespace mt::qpar {
    // used to iterate the nodes in a tree from a parallel for
    class Incrementer {
    public:
        Incrementer(const mt::Nodes& nodes, const Nodes::const_reverse_iterator& delimiter, const std::size_t& startPos, const std::size_t& delta);

        Incrementer& operator++();

        inline const mt::Nodes::const_reverse_iterator& get() { return this->cursor; };

    private:
        Nodes::const_reverse_iterator end;
        Nodes::const_reverse_iterator cursor;
        std::size_t delta;
    };
}

#endif