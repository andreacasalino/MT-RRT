/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <deque>
#include <span>

namespace mt_rrt {
template <typename T> class ObjectPool {
public:
  ObjectPool() = default;

  ObjectPool(const ObjectPool &) = delete;
  ObjectPool &operator=(const ObjectPool &) = delete;

  ObjectPool(ObjectPool &&) = delete;
  ObjectPool &operator=(ObjectPool &&) = delete;

  T &push(T &&to_add) { return pool_.emplace_back(std::forward<T>(to_add)); }

  void push(std::span<const T> to_add) {
    pool_.insert(pool_.end(), to_add.begin(), to_add.end());
  }

private:
  std::deque<T> pool_;
};
} // namespace mt_rrt
