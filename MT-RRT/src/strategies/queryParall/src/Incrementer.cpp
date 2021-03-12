/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/Incrementer.h"

namespace mt::solver::qpar {
    Incrementer::Incrementer(const Nodes::const_reverse_iterator& rend, const Nodes::const_reverse_iterator& rbegin, const std::size_t& startPos, const std::size_t& delta)
        : end(rend)
        , cursor(rbegin)
        , delta(startPos) {
        ++(*this);
        this->delta = delta;
    };

    Incrementer& Incrementer::operator++() {
        std::size_t k = 0;
        while ((k < this->delta) && (this->cursor != this->end)) {
            ++this->cursor;
            ++k;
        }
        return *this;
    };
}