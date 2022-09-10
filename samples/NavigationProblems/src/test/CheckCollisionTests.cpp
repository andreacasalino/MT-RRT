#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <NavigationProblem.h>
#include <NavigationProblemJson.h>

#ifdef TEST_LOGGING
#include <Logger.h>
#endif

#ifdef TEST_LOGGING
namespace {
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
} // namespace
#endif

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
    auto state = GENERATE(State{0.1f, 0.f, 0.f}, State{0.f, 0.1f, 0.f},
                          State{-0.1f, 0.f, 0.f}, State{0.f, -0.1f, 0.f},
                          State{0.1f, 0.f, PI_HALF}, State{0.f, 0.1f, PI});

    bool collisionFlag = cart.isCollisionPresent(obstacle, state);
    CHECK(collisionFlag);

#ifdef TEST_LOGGING
    nlohmann::json log_json = log_case(cart, state, obstacle);
    log_json["collides"] = collisionFlag;
    Logger::log("box_segment_collisions", log_json, PYTHON_SCRIPT);
#endif
  }

  SECTION("no collision expected") {
    auto state = GENERATE(State{2.1f, 0.f, 0.f}, State{0.f, 2.1f, PI_HALF},
                          State{2.5f, 2.5f, to_rad(135)},
                          State{-2.5f, -2.5f, to_rad(135)});

    bool collisionFlag = cart.isCollisionPresent(obstacle, state);
    CHECK_FALSE(collisionFlag);

#ifdef TEST_LOGGING
    nlohmann::json log_json = log_case(cart, state, obstacle);
    log_json["collides"] = collisionFlag;
    Logger::log("box_segment_collisions", log_json, PYTHON_SCRIPT);
#endif
  }
}
