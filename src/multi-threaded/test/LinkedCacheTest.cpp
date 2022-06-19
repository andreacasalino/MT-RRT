#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "LinkedCache.h"

namespace {
struct IndexRange {
  IndexRange(const std::size_t to_skip, const std::size_t size) {
    for (std::size_t j = 0; j < size; ++j) {
      if (j == to_skip) {
        continue;
      }
      indices.push_back(j);
    }
  }

  std::size_t get() const { return indices[k]; };
  std::size_t getCounter() const { return k; }

  bool operator()() const { return k < indices.size(); }

  IndexRange &operator++() {
    ++k;
    return *this;
  }

  std::vector<std::size_t> indices;
  std::size_t k = 0;
};
} // namespace

TEST_CASE("Linked caches", mt_rrt::merge(TEST_TAG, "[linked_cache]")) {
  using namespace mt_rrt;

  auto size = GENERATE(3, 4, 7);

  struct Message {
    std::size_t sender;
    std::size_t receiver;
  };

  using MessageCachePtr = std::unique_ptr<LinkedCache<Message>>;

  std::vector<MessageCachePtr> caches;
  for (std::size_t k = 0; k < size; ++k) {
    caches.emplace_back(std::make_unique<LinkedCache<Message>>());
  }

  LinkedCache<Message>::link_caches(caches.begin(), caches.end());

  for (const auto &cache : caches) {
    REQUIRE(cache->incoming_caches.size() == (size - 1));
    REQUIRE(cache->outgoing_caches.size() == (size - 1));
  }

  for (std::size_t sender = 0; sender < size; ++sender) {
    for (IndexRange receiver_range(sender, size); receiver_range();
         ++receiver_range) {
      caches[sender]->outgoing_caches[receiver_range.getCounter()]->push_back(
          Message{sender, receiver_range.get()});
    }
  }

  for (std::size_t receiver = 0; receiver < size; ++receiver) {
    for (IndexRange sender_range(receiver, size); sender_range();
         ++sender_range) {
      for (const auto &message :
           *caches[receiver]->incoming_caches[sender_range.getCounter()]) {
        CHECK(message.sender == sender_range.get());
        CHECK(message.receiver == receiver);
      }
    }
  }

  SECTION("drain incoming caches") {
    caches.front()->drainIncomingCaches([](const Message &) {});
    for (const auto &cache : caches.front()->incoming_caches) {
      if (nullptr == cache) {
        continue;
      }
      CHECK(cache->empty());
    }
    for (const auto &cache : caches.back()->incoming_caches) {
      if (nullptr == cache) {
        continue;
      }
      CHECK_FALSE(cache->empty());
    }
  }
}
