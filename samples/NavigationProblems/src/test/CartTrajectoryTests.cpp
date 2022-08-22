#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <NavigationProblem.h>
#include <NavigationProblemJson.h>

#ifdef TEST_LOGGING
#include <Logger.h>
#endif

#include <optional>

#ifdef TEST_LOGGING
namespace {
nlohmann::json log_case(const mt_rrt::State &start, const mt_rrt::State &end,
                        const mt_rrt::samples::CartTrajectoryInfo &trj) {
  nlohmann::json result;
  result["start"] = start;
  result["end"] = end;
  struct Visitor {
    nlohmann::json &result;

    void operator()(const mt_rrt::samples::Blended &arc) const {
      result["type"] = "arc";
      result["start"] = arc.start;
      result["end"] = arc.end;
      result["ray"] = arc.ray;
      result["arc_begin"] = arc.arc_begin;
      result["arc_end"] = arc.arc_end;
    }

    void operator()(const mt_rrt::samples::TrivialLine &line) const {
      result["type"] = "line";
      result["start"] = line.start;
      result["end"] = line.end;
    }
  } visitor{result["traj"]};
  std::visit(visitor, trj);
  return result;
}

nlohmann::json log_case(const mt_rrt::State &start, const mt_rrt::State &end,
                        mt_rrt::Trajectory &trj) {
  nlohmann::json result;
  result["start"] = start;
  result["end"] = end;

  std::vector<mt_rrt::State> states;
  mt_rrt::AdvanceInfo advance_info = mt_rrt::AdvanceInfo::advanced;
  while (advance_info == mt_rrt::AdvanceInfo::advanced) {
    states.push_back(trj.getState());
    advance_info = trj.advance();
  }
  result["traj"] = states;

  return result;
}

static const std::string PYTHON_SCRIPT =
    mt_rrt::merge(TEST_FOLDER, "CartTrajectoryTests.py");

static constexpr float TOLL = 1e-3f;

bool almost_equal(const float *a, const float *b, std::size_t size) {
  for (std::size_t k = 0; k < size; ++k) {
    if (std::abs(a[k] - b[k]) > TOLL) {
      return false;
    }
  }
  return true;
}

bool almost_equal(const mt_rrt::State &a, const mt_rrt::State &b) {
  return almost_equal(a.data(), b.data(), 3);
}

bool almost_equal(const mt_rrt::samples::Point &a,
                  const mt_rrt::samples::Point &b) {
  return almost_equal(a.data(), b.data(), 2);
}
} // namespace
#endif

TEST_CASE("Trivial line", mt_rrt::merge(TEST_TAG, "[cart_trajectory]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  CartSteerLimits limits;

  SECTION("aligned on different lines") {
    const State start = State{0, -0.5f, to_rad(65)};
    const State end = State{3.f, 3.f, to_rad(65)};
    auto traj = compute_cart_trajectory_info(start, end, limits);
    CHECK_FALSE(traj);
  }

  SECTION("aligned on same line, but opposite directions") {
    const State start = State{0, 0, to_rad(45.f)};
    const State end = State{1.f, 1.f, to_rad(-135.f)};
    auto traj = compute_cart_trajectory_info(start, end, limits);
    CHECK_FALSE(traj);
  }

  SECTION("aligned on same line, but bad positions") {
    const State start = State{1.f, 1.f, to_rad(45)};
    const State end = State{0, 0, to_rad(45)};
    auto traj = compute_cart_trajectory_info(start, end, limits);
    CHECK_FALSE(traj);
  }

  SECTION("alinged: trivial line") {
    const State start = State{0, 0, to_rad(45)};
    const State end = State{1.f, 1.f, to_rad(45)};
    auto traj = compute_cart_trajectory_info(start, end, limits);
    REQUIRE(traj);
    CHECK(std::holds_alternative<TrivialLine>(traj.value()));
    auto &as_line = std::get<TrivialLine>(traj.value());

#ifdef TEST_LOGGING
    nlohmann::json log_json = log_case(start, end, traj.value());
    Logger::log("trivial_line", log_json, PYTHON_SCRIPT);
#endif

    CHECK(almost_equal(as_line.start, start));
    CHECK(almost_equal(as_line.end, end));
  }
}

namespace {
class TrajTestCaseBase {
public:
  const mt_rrt::State &getStart() const { return start; };
  const mt_rrt::State &getEnd() const { return end; };

protected:
  TrajTestCaseBase(float angle, float distance, bool possible_or_impossible) {
    start = {0, -distance, mt_rrt::utils::PI_HALF};
    end = {distance * cosf(angle), distance * sinf(angle), angle};
    if (not possible_or_impossible) {
      end[2] += mt_rrt::utils::PI;
    }
  }

  mt_rrt::State start;
  mt_rrt::State end;
};

class TrajTestCase : public TrajTestCaseBase {
public:
  TrajTestCase(float angle, float distance,
               const mt_rrt::samples::CartSteerLimits &limits)
      : TrajTestCaseBase(angle, distance, true) {
    auto info = expected_result.emplace(mt_rrt::samples::Blended{start, end});
    const auto gamma = 0.5f * (-mt_rrt::utils::PI_HALF + angle);
    const float angle_delta = std::abs(angle - gamma);
    const float angle_delta_tan = std::abs(tanf(angle_delta));
    info.ray = distance * angle_delta_tan;
    info.ray = std::min(info.ray, limits.maxRadius());
    const auto diagonal = std::abs(info.ray / sinf(angle_delta));
    info.center = {diagonal * cosf(gamma), diagonal * sinf(gamma)};
    const auto d = info.ray / angle_delta_tan;
    info.arc_begin = {0, -d};
    info.arc_end = {d * cosf(angle), d * sinf(angle)};
  }

  bool almostEqualToExpected(const mt_rrt::samples::Blended &pieces) const {
    return almost_equal(pieces.start, expected_result->start) &&
           almost_equal(pieces.end, expected_result->end) &&
           (std::abs(pieces.ray - expected_result->ray) < TOLL) &&
           almost_equal(pieces.center, expected_result->center) &&
           almost_equal(pieces.arc_begin, expected_result->arc_begin) &&
           almost_equal(pieces.arc_end, expected_result->arc_end);
  }

private:
  std::optional<mt_rrt::samples::Blended> expected_result;
};

class TrajTestCaseImpossible : public TrajTestCaseBase {
public:
  TrajTestCaseImpossible(float angle, float distance)
      : TrajTestCaseBase(angle, distance, false) {}
};
} // namespace

TEST_CASE("Blended arc", mt_rrt::merge(TEST_TAG, "[cart_trajectory]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  CartSteerLimits limits(0.2f, 1.f);

  auto angle = GENERATE(to_rad(0), to_rad(-45.f), to_rad(45.f), to_rad(135.f),
                        to_rad(-135.f));

  SECTION("") {
    auto distance = GENERATE(0.5f, 1.f, 2.f);

    SECTION("impossible") {
      TrajTestCaseImpossible test_case(angle, distance);
      const auto &start = test_case.getStart();
      const auto &end = test_case.getEnd();

      auto traj = compute_cart_trajectory_info(start, end, limits);
      CHECK_FALSE(traj);
    }

    SECTION("possible") {
      TrajTestCase test_case(angle, distance, limits);
      const auto &start = test_case.getStart();
      const auto &end = test_case.getEnd();

      auto traj = compute_cart_trajectory_info(start, end, limits);
      REQUIRE(traj);
      CHECK(std::holds_alternative<Blended>(traj.value()));
      auto &as_arc = std::get<Blended>(traj.value());

#ifdef TEST_LOGGING
      std::stringstream log_name;
      log_name << "pieces_" << to_grad(angle) << "_degree";
      nlohmann::json log_json = log_case(start, end, traj.value());
      Logger::log(log_name.str(), log_json, PYTHON_SCRIPT);
#endif

      CHECK(test_case.almostEqualToExpected(as_arc));
    }
  }

  SECTION("impossible due to steer limits") {
    TrajTestCaseImpossible test_case(angle, 0.1f);
    const auto &start = test_case.getStart();
    const auto &end = test_case.getEnd();

    auto traj = compute_cart_trajectory_info(start, end, limits);
    CHECK_FALSE(traj);
  }
}

// compute interpolated and checks

// check total cost
