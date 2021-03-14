/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_ITERATOR_H
#define MT_RRT_TREE_ITERATOR_H

#include <Tree.h>

namespace mt::solver::qpar {
    class TreeIterator {
    public:
        TreeIterator(const mt::Tree& tree, const std::size_t& startPos, const std::size_t& delta);

        TreeIterator& operator++();

        inline const mt::Nodes::const_reverse_iterator& get() const { return this->cursor; };
        inline const mt::Nodes::const_reverse_iterator& end() const { return this->rend; };

    private:
        const Nodes::const_reverse_iterator rend;
        Nodes::const_reverse_iterator cursor;
        std::size_t delta;
    };
}

#endif