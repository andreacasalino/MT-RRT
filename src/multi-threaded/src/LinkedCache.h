/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Error.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace mt_rrt {
class LinkedIndicesRange {
public:
  LinkedIndicesRange(std::size_t to_skip) : to_skip(to_skip), cursor(0) {
    if (0 == to_skip) {
      ++cursor;
    }
  }

  LinkedIndicesRange &operator++() {
    ++cursor;
    if (cursor == to_skip) {
      ++cursor;
    }
    return *this;
  }

  operator std::size_t() const { return cursor; }

private:
  const std::size_t to_skip;
  std::size_t cursor;
};

template <typename T> class LinkedCache {
public:
  LinkedCache() = default;

  template <typename Iterator>
  static void link_caches(Iterator begin, Iterator end) {
    std::vector<LinkedCache<T> *> subjects;
    std::for_each(begin, end, [&subjects](auto &element) {
      auto *to_add = dynamic_cast<LinkedCache<T> *>(element.get());
      if (nullptr == to_add) {
        throw Error{"invalid cache to link"};
      }
      subjects.push_back(to_add);
    });

    const std::size_t size = subjects.size();
    std::vector<std::vector<CachePtr>> caches;
    for (std::size_t r = 0; r < size; ++r) {
      auto &row = caches.emplace_back();
      for (std::size_t c = 0; c < size; ++c) {
        row.push_back(std::make_shared<Cache>());
      }
    }
    for (std::size_t k = 0; k < size; ++k) {
      auto *subject = subjects[k];
      for (LinkedIndicesRange l(k); l < size; ++l) {
        subject->outgoing_caches.push_back(caches[k][l]);
        subject->incoming_caches.push_back(caches[l][k]);
      }
    }
  }

  template <typename Predicate> void drainIncomingCaches(Predicate predicate) {
    for (const auto &cache : incoming_caches) {
      for (const auto &element : *cache) {
        predicate(element);
      }
      cache->clear();
    }
  }

  using Cache = std::vector<T>;
  using CachePtr = std::shared_ptr<Cache>;

  std::vector<CachePtr> outgoing_caches;
  std::vector<CachePtr> incoming_caches;
};
} // namespace mt_rrt
