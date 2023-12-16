#include <gtest/gtest.h>

#include <NavigationProblem.h>
#include <NavigationProblemConversions.h>

using namespace mt_rrt;
using namespace mt_rrt::navigation;

namespace {
using State = std::array<float, 3>;

class TestCaseLog {
public:
  TestCaseLog(const std::string &tag, const Cart &cart) : tag{tag} {
    title = "case-" + std::to_string(getCount(tag));
    to_json(log["cart"], cart);
    poses = nlohmann::json::array();
    obstacles = nlohmann::json::array();
  }
  ~TestCaseLog() { Logger::get().add(tag, title, log); }

  void addPose(const View &pose) { poses.emplace_back() = pose.convert(); }

  void addObstacle(const geom::Sphere &sphere) {
    auto &added = obstacles.emplace_back();
    to_json(added, sphere);
    added["type"] = ObstacleTrait<geom::Sphere>::typeStr();
  }

  static void log_simple(const Cart &cart, const View &cart_state,
                         const geom::Sphere &sphere, const std::string &tag) {
    TestCaseLog res{tag, cart};
    res.addObstacle(sphere);
    res.addPose(cart_state);
  }

private:
  static std::size_t getCount(const std::string &tag) {
    static std::unordered_map<std::string, std::size_t> logs =
        std::unordered_map<std::string, std::size_t>{};
    std::size_t res;
    if (auto it = logs.find(tag); it == logs.end()) {
      logs.emplace(tag, 1);
      res = 1;
    } else {
      res = ++it->second;
    }
    return res;
  }

  std::string tag;
  std::string title;

  nlohmann::json log;
  nlohmann::json &poses = log["poses"];
  nlohmann::json &obstacles = log["obstacles"];
};

struct CartCollisionTestData {
  CartCollisionTestData() {
    obstacle.ray.set(1.f);
    obstacle.center = geom::Point{0, 0};
  }

  Cart cart{1.f, 2.f, CartSteerLimits{}};
  geom::Sphere obstacle;
};

struct Combine {
  geom::Point res;

  template <typename... Points>
  Combine(const geom::Point &a, const geom::Point &b, const Points &...points)
      : res{geom::sum(a, b)} {
    (this->add(points), ...);
  }

  State state(float angle) const {
    return State{res.data()[0], res.data()[1], angle};
  }

  void add(const geom::Point &point) { res = geom::sum(res, point); }
};

} // namespace

using CartCollisionTestWithCollisionFixture = ::testing::TestWithParam<State>;

TEST_P(CartCollisionTestWithCollisionFixture, collision_expected) {
  const auto &state = GetParam();

  auto &&[cart, obstacle] = CartCollisionTestData{};

  View state_view{state.data(), 3};
  bool collisionFlag = cart.isCollisionPresent(obstacle, state_view);
  EXPECT_TRUE(collisionFlag);
  TestCaseLog::log_simple(cart, state_view, obstacle, "with-collision");
}

INSTANTIATE_TEST_CASE_P(
    CartCollisionTestsWithCollision, CartCollisionTestWithCollisionFixture,
    ::testing::Values(
        State{0.1f, 0.f, 0.f}, State{0.f, 0.1f, 0.f}, State{-0.1f, 0.f, 0.f},
        State{0.f, -0.1f, 0.f}, State{0.1f, 0.f, geom::PI_HALF},
        State{0.f, 0.1f, geom::PI},
        Combine{geom::Point{0.6f, 0.2f},
                geom::Versor{geom::to_rad(30.f)}.asPoint(),
                geom::Versor{geom::to_rad(30.f + 90.f)}.asPointScaled(0.5f)}
            .state(geom::to_rad(30.f))));

using CartCollisionTestNoCollisionFixture = ::testing::TestWithParam<State>;

TEST_P(CartCollisionTestNoCollisionFixture, no_collision_expected) {
  const auto &state = GetParam();

  auto &&[cart, obstacle] = CartCollisionTestData{};

  View state_view{state.data(), 3};
  bool collisionFlag = cart.isCollisionPresent(obstacle, state_view);
  EXPECT_FALSE(collisionFlag);
  TestCaseLog::log_simple(cart, state_view, obstacle, "no-collision");
}

INSTANTIATE_TEST_CASE_P(
    CartCollisionTestsNoCollision, CartCollisionTestNoCollisionFixture,
    ::testing::Values(
        State{2.1f, 0.f, 0.f}, State{0.f, 2.1f, geom::PI_HALF},
        State{2.5f, 2.5f, geom::to_rad(135)},
        State{-2.5f, -2.5f, geom::to_rad(135)},
        Combine{geom::Point{0.85f, 0.85f},
                geom::Versor{geom::to_rad(30.f)}.asPoint(),
                geom::Versor{geom::to_rad(30.f + 90.f)}.asPointScaled(0.5f)}
            .state(geom::to_rad(30.f))));
