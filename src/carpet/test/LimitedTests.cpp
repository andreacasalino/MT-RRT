#include <gtest/gtest.h>

#include <MT-RRT/Error.h>
#include <MT-RRT/Limited.h>

#include <memory>

namespace {
using Lim = mt_rrt::Limited<float, 2.f, 5.f>;
} // namespace

TEST(LimitedTest, limited) {

  EXPECT_ANY_THROW(Lim{1.f});
  EXPECT_ANY_THROW(Lim{6.f});

  EXPECT_NO_THROW(Lim{2.f});
  EXPECT_NO_THROW(Lim{5.f});

  Lim val{3.f};

  EXPECT_NO_THROW(val.set(4.f));
  EXPECT_ANY_THROW(Lim{1.f});
}
