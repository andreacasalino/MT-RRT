#include <gtest/gtest.h>

#include <Transform.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.0001f; }

bool almost_equal(const std::vector<float> &a, const std::vector<float> &b) {
  if ((a.size() != 2) || (b.size() != 2)) {
    throw std::runtime_error{"Invalid std::vector<float>"};
  }
  for (std::size_t k = 0; k < 2; ++k) {
    if (!almost_equal(a[k], b[k])) {
      return false;
    }
  }
  return true;
}

bool almost_equal(const Point &a, const Point &b) {
  for (std::size_t k = 0; k < 2; ++k) {
    if (!almost_equal(a.data()[k], b.data()[k])) {
      return false;
    }
  }
  return true;
}
} // namespace

TEST(TransformTest, seen_from_relative) {
  {
    SCOPED_TRACE("no rotation neither traslation");

    Transform t(std::nullopt, std::nullopt);

    for (const auto &point : std::vector<Point>{Point{0, 0}, Point{1, 1},
                                                Point{1, 2}, Point{-2, 1}}) {
      auto point_from_relative = t.seenFromRelativeFrame(point);
      EXPECT_TRUE(almost_equal(point, point_from_relative));
    }
  }

  {
    SCOPED_TRACE("only rotation");

    Transform t(to_rad(45.f), std::nullopt);

    Point point{1.f, 1.f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    EXPECT_TRUE(almost_equal(point_from_relative, Point{sqrtf(2.f), 0}));
  }

  {
    SCOPED_TRACE("traslation and rotation");

    Transform t(to_rad(45.f), Point{0, 1.5f});

    Point point{1.f, 2.5f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    EXPECT_TRUE(almost_equal(point_from_relative, Point{sqrtf(2.f), 0}));
  }
}

TEST(TransformTest, combined_transform) {
  {
    SCOPED_TRACE("combine with only rotations");

    const float angle = to_rad(10.f);
    Transform t(angle, std::nullopt);
    Transform t2 = Transform::combine(t, t);

    EXPECT_TRUE(almost_equal(t2.getTraslation(), Point{0, 0}));

    auto t2_angle = t2.getAngle();
    EXPECT_TRUE(almost_equal(t2_angle, angle * 2.f));
  }

  {
    SCOPED_TRACE("combine general");

    auto t = Transform::combine(Transform{to_rad(90.f / 4.f), Point{0, 1.5f}},
                                Transform{to_rad(90.f / 4.f), std::nullopt});

    Point point{1.f, 2.5f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    EXPECT_TRUE(almost_equal(point_from_relative, Point{sqrtf(2.f), 0}));
  }
}
