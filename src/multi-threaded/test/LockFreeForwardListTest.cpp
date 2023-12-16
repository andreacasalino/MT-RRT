#include <gtest/gtest.h>

#include <MT-RRT/LockFreeForwardList.h>
#include <MT-RRT/Strings.h>

#include "Utils.h"

using namespace mt_rrt;

namespace {
std::vector<Person> convert(LockFreeForwardList<Person> &list) {
  std::vector<Person> res;
  list.forEach([&res](const Person &element) {
    res.emplace_back(element.getName(), element.getSurName());
  });
  return res;
}
} // namespace

TEST(LockFreeListTest, push_pop_from_same_thread) {
  LockFreeForwardList<Person> list{"foo", "bla"};
  std::size_t times = 10;
  for (std::size_t k = 0; k < times - 1; ++k) {
    list.emplace_back("foo", "bla");
  }

  std::vector<Person> expected;
  for (std::size_t k = 0; k < times; ++k) {
    expected.emplace_back("foo", "bla");
  }
  EXPECT_EQ(convert(list), expected);
}

TEST(LockFreeListTest, concurrent_push) {
  LockFreeForwardList<Person> list{"foo", "bla"};
  std::size_t times = 50;

  auto producer = [&] {
    for (std::size_t k = 0; k < times; ++k) {
      list.emplace_back("foo", "bla");
    }
  };

  ParallelRegion::execute(producer, producer);

  std::vector<Person> expected;
  for (std::size_t k = 0; k < times * 2 + 1; ++k) {
    expected.emplace_back("foo", "bla");
  }
  EXPECT_EQ(convert(list), expected);
}

TEST(LockFreeListTest, push_while_looping) {
  LockFreeForwardList<Person> list{"foo", "bla"};
  std::size_t insertions = 40;

  std::atomic_bool barrier = true;
  std::size_t list_snapshot_size;
  ParallelRegion::execute(
      [&] {
        for (std::size_t k = 0; k < insertions / 2; ++k) {
          list.emplace_back("foo", "bla");
        }
        barrier = false;
        for (std::size_t k = 0; k < insertions / 2; ++k) {
          list.emplace_back("foo", "bla");
        }
      },
      [&] {
        while (barrier.load()) {
        }
        std::size_t size = 0;
        list.forEach([&](const Person &person) { ++list_snapshot_size; });
      });

  EXPECT_TRUE(insertions / 2 <= list_snapshot_size);
}
