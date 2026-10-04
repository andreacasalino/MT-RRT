#include <gtest/gtest.h>

#include <TrivialProblem.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;
using namespace mt_rrt::problem_trivial;

class TrivialProblemTest : public ::testing::Test {
protected:
  problem_trivial::Connector connector{
      std::make_shared<Boxes>(
          Boxes{Box{PointAllocated{-1.f, -1.f}, PointAllocated{1.f, 1.f}}}),
      SteerIterations{10000}};

  std::vector<float> steer_buffer;
};

TEST_F(TrivialProblemTest, no_steer_as_blocked) {
  std::vector<float> start{
      -1.f - problem_trivial::Connector::STEER_DEGREE * 0.1f,
      -1.f - problem_trivial::Connector::STEER_DEGREE * 0.1f};
  std::vector<float> end{2.f, 2.f};

  auto steered = connector.steer(start, end, steer_buffer);

  EXPECT_FALSE(steered);
}

TEST_F(TrivialProblemTest, advanced) {
  std::vector<float> start{-2.f, -2.f};
  std::vector<float> end{2.f, 2.f};

  auto steered = connector.steer(start, end, steer_buffer);

  ASSERT_TRUE(steered);

  // should have been blocked to a coordinate similar to (-val, -val)
  EXPECT_FALSE(steered->target_was_reached);
  ASSERT_EQ(steer_buffer.size(), 2);
  EXPECT_TRUE(fabs(steer_buffer[0] - steer_buffer[1]) < 1e-4f);
  EXPECT_TRUE(steer_buffer[0] < -1.f);
}

TEST_F(TrivialProblemTest, target_reached) {
  std::vector<float> start{-2.f, -2.f};
  std::vector<float> end{-2.f, 2.f};

  auto steered = connector.steer(start, end, steer_buffer);

  ASSERT_TRUE(steered);
  // target should have been reached
  EXPECT_TRUE(steered->target_was_reached);
}
