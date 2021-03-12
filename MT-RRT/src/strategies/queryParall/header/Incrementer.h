/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_INCREMENTER_H
#define MT_RRT_INCREMENTER_H

#include <Tree.h>

namespace mt::solver::qpar {
    // used to iterate the nodes in a tree from a parallel for
    class Incrementer {
    public:
        Incrementer(const Nodes::const_reverse_iterator& rend, const Nodes::const_reverse_iterator& rbegin, const std::size_t& startPos, const std::size_t& delta);

        Incrementer& operator++();

        inline const mt::Nodes::const_reverse_iterator& get() const { return this->cursor; };
        inline const mt::Nodes::const_reverse_iterator& getEnd() const { return this->end; };

    private:
        Nodes::const_reverse_iterator end;
        Nodes::const_reverse_iterator cursor;
        std::size_t delta;
    };
}

#endif