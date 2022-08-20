#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <TrivialProblem.h>

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.0001f; }

bool almost_equal(const mt_rrt::State &a, const mt_rrt::State &b) {
  if ((a.size() != 2) || (b.size() != 2)) {
    throw std::runtime_error{"Invalid state"};
  }
  for (std::size_t k = 0; k < 2; ++k) {
    if (!almost_equal(a[k], b[k])) {
      return false;
    }
  }
  return true;
}

bool almost_equal(const mt_rrt::samples::Transform::Traslation &a,
                  const mt_rrt::samples::Transform::Traslation &b) {
  for (std::size_t k = 0; k < 2; ++k) {
    if (!almost_equal(a[k], b[k])) {
      return false;
    }
  }
  return true;
}
} // namespace

TEST_CASE("check seen from relative", mt_rrt::merge(TEST_TAG, "[transform]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::samples;

  SECTION("no rotation neither traslation") {
    Transform t(std::nullopt, std::nullopt);

    auto point = GENERATE(State{0, 0}, State{1, 1}, State{1, 2}, State{-2, 1});

    auto point_from_relative = t.seenFromRelativeFrame(point);
    CHECK(almost_equal(point, point_from_relative));
  }

  SECTION("only rotation") {
    Transform t(to_rad(45.f), std::nullopt);

    State point = State{1.f, 1.f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    CHECK(almost_equal(point_from_relative, State{sqrtf(2.f), 0}));
  }

  SECTION("traslation and rotation") {
    Transform t(to_rad(45.f), Transform::Traslation{0, 1.5f});

    State point = State{1.f, 2.5f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    CHECK(almost_equal(point_from_relative, State{sqrtf(2.f), 0}));
  }
}

TEST_CASE("check combine transform", mt_rrt::merge(TEST_TAG, "[transform]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::samples;

  SECTION("combine with only rotations") {
    const float angle = to_rad(10.f);
    Transform t(angle, std::nullopt);
    Transform t2 = Transform::combine(t, t);

    CHECK(almost_equal(t2.getTraslation(), Transform::Traslation{0, 0}));

    auto t2_angle = t2.getAngle();
    CHECK(almost_equal(t2_angle, angle * 2.f));
  }

  SECTION("combine general") {
    auto t = Transform::combine(
        Transform{to_rad(90.f / 4.f), Transform::Traslation{0, 1.5f}},
        Transform{to_rad(90.f / 4.f), std::nullopt});

    State point = State{1.f, 2.5f};

    auto point_from_relative = t.seenFromRelativeFrame(point);
    CHECK(almost_equal(point_from_relative, State{sqrtf(2.f), 0}));
  }
}
