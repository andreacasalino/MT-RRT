#include <gtest/gtest.h>

#include <TrivialProblem.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;
using namespace mt_rrt::trivial;

class TrivialProblemTest : public ::testing::Test {
protected:
  void SetUp() { connector.emplace(Boxes{Box{{-1.f, -1.f}, {1.f, 1.f}}}); }

  std::optional<TrivialProblemConnector> connector;
  NodesAllocator tree;
};

TEST_F(TrivialProblemTest, no_steer_as_blocked) {
  SteerIterations trials{10000};
  auto &added = tree.emplace_back(
      std::vector<float>{-1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f,
                         -1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f});
  auto steered = connector->steer(added, std::vector<float>{2.f, 2.f}, trials);
  EXPECT_FALSE(steered);
}

TEST_F(TrivialProblemTest, advanced) {
  auto &start = tree.emplace_back(std::vector<float>{-2.f, -2.f});

  SteerIterations trials{10000};
  auto steered = connector->steer(start, std::vector<float>{2.f, 2.f}, trials);

  ASSERT_TRUE(steered);

  // should have been blocked to a coordinate similar to (-val, -val)
  EXPECT_FALSE(steered->target_is_reached);
  const auto &node = steered->node;
  const auto &state = node.state();
  EXPECT_EQ(node.getParent(), &start);
  ASSERT_EQ(state.size, 2);
  EXPECT_TRUE(fabs(state.data[0] - state.data[1]) < 1e-4f);
  EXPECT_TRUE(state.data[0] < -1.f);
}

TEST_F(TrivialProblemTest, target_reached) {
  auto &start = tree.emplace_back(std::vector<float>{-2.f, -2.f});

  std::vector<float> target{-2.f, 2.f};
  SteerIterations trials{10000};
  auto steered = connector->steer(start, target, trials);

  ASSERT_TRUE(steered);
  // target should have been reached
  EXPECT_TRUE(steered->target_is_reached);
  const auto &node = steered->node;
  const auto &state = node.state();
  EXPECT_EQ(node.getParent(), &start);
  EXPECT_EQ(node.state().convert(), target);
}
