/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeIterator.h"

namespace mt::solver::qpar {
    TreeIterator::TreeIterator(const mt::Tree& tree, const std::size_t& startPos, const std::size_t& delta)
        : rend(tree.rend())
        , cursor(tree.rbegin())
        , delta(startPos) {
        ++(*this);
        this->delta = delta;
    };

    TreeIterator& TreeIterator::operator++() {
        std::size_t k = 0;
        while ((k < this->delta) && (this->cursor != this->rend)) {
            ++this->cursor;
            ++k;
        }
        return *this;
    };
}