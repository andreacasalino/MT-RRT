#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <NavigationProblem.h>
#include <NavigationProblemJson.h>

#ifdef TEST_LOGGING
#include <IO.h>
#endif

namespace {
#ifdef TEST_LOGGING
nlohmann::json log_case(const mt_rrt::samples::Cart &cart,
                        const mt_rrt::State &cart_state,
                        const mt_rrt::samples::Sphere &sphere) {
  nlohmann::json result;
  mt_rrt::samples::to_json(result["cart"], cart);
  mt_rrt::samples::to_json(result["sphere"], sphere);
  result["state"] = cart_state;
  return result;
}

static const std::string PYTHON_SCRIPT =
    mt_rrt::merge(TEST_FOLDER, "CheckCollisionTests.py");

#endif

mt_rrt::State to_state(const mt_rrt::utils::Point &location,
                       float orientation) {
  return mt_rrt::State{location[0], location[1], orientation};
}

struct DirectionAndScale {
  mt_rrt::utils::Versor direction;
  float scale;
};
DirectionAndScale make_dir_and_scale(float angle, float scale = 1.f) {
  return DirectionAndScale{mt_rrt::utils::Versor{angle}, scale};
}

namespace detail {
void combine_(mt_rrt::utils::Point &recipient,
              const mt_rrt::utils::Point &to_add) {
  mt_rrt::utils::add(recipient, to_add);
}

void combine_(mt_rrt::utils::Point &recipient,
              const DirectionAndScale &to_add) {
  mt_rrt::utils::add(recipient, to_add.direction.asPoint(), to_add.scale);
}

template <typename T, typename... Args>
void combine_(mt_rrt::utils::Point &recipient, const T &first,
              const Args &...args) {
  combine_(recipient, first);
  combine_(recipient, args...);
}
} // namespace detail

template <typename T, typename... Args>
mt_rrt::utils::Point combine(const T &first, const Args &...args) {
  mt_rrt::utils::Point result = {0, 0};
  detail::combine_(result, first, args...);
  return result;
}
} // namespace

TEST_CASE("check collision check cart-sphere",
          mt_rrt::merge(TEST_TAG, "[collision_check]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  CartSteerLimits limits;
  Cart cart(1.f, 2.f, limits);

  Sphere obstacle;
  obstacle.ray.set(1.f);
  obstacle.center = Point{0, 0};

  SECTION("collision expected") {
    auto state = GENERATE(
        State{0.1f, 0.f, 0.f}, State{0.f, 0.1f, 0.f}, State{-0.1f, 0.f, 0.f},
        State{0.f, -0.1f, 0.f}, State{0.1f, 0.f, PI_HALF}, State{0.f, 0.1f, PI},
        to_state(combine(Point{0.6f, 0.2f},
                         make_dir_and_scale(to_rad(30.f), 1.f),
                         make_dir_and_scale(to_rad(30.f + 90.f), 0.5f)),
                 to_rad(30.f)));

    bool collisionFlag = cart.isCollisionPresent(obstacle, state);
    CHECK(collisionFlag);

#ifdef TEST_LOGGING
    mt_rrt::utils::Logger::Log to_log;
    to_log.tag = "box_segment_collisions";
    to_log.python_visualizer = PYTHON_SCRIPT;
    to_log.content = log_case(cart, state, obstacle);
    to_log.content["collides"] = collisionFlag;
    mt_rrt::utils::Logger::log(to_log);
#endif
  }

  SECTION("no collision expected") {
    auto state = GENERATE(
        State{2.1f, 0.f, 0.f}, State{0.f, 2.1f, PI_HALF},
        State{2.5f, 2.5f, to_rad(135)}, State{-2.5f, -2.5f, to_rad(135)},
        to_state(combine(Point{0.85f, 0.85f},
                         make_dir_and_scale(to_rad(30.f), 1.f),
                         make_dir_and_scale(to_rad(30.f + 90.f), 0.5f)),
                 to_rad(30.f)));

    bool collisionFlag = cart.isCollisionPresent(obstacle, state);
    CHECK_FALSE(collisionFlag);

#ifdef TEST_LOGGING
    mt_rrt::utils::Logger::Log to_log;
    to_log.tag = "box_segment_collisions";
    to_log.python_visualizer = PYTHON_SCRIPT;
    to_log.content = log_case(cart, state, obstacle);
    to_log.content["collides"] = collisionFlag;
    mt_rrt::utils::Logger::log(to_log);
#endif
  }
}
