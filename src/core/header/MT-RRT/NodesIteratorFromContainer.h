/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>

namespace mt_rrt {
template <typename Container> class NodesIteratorFromContainer {
public:
  NodesIteratorFromContainer(const Container &container)
      : size_{container.size()}, current_{container.begin()},
        end_{container.end()} {}

  std::size_t size() const { return size_; }

  const Node &next() {
    if (current_ == end_) {
      return nullptr;
    } else {
      const Node &res = *current_;
      ++current_;
      return res;
    }
  }

private:
  using Iter = typename Container::const_iterator;
  std::size_t size_;
  Iter current_;
  Iter end_;
};
} // namespace mt_rrt
