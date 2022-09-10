#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <NavigationProblem.h>
#include <NavigationProblemJson.h>

#ifdef TEST_LOGGING
#include <Logger.h>
#endif

namespace {
#ifdef TEST_LOGGING
nlohmann::json log_case(const mt_rrt::State &start, const mt_rrt::State &end,
                        const mt_rrt::samples::CartTrajectoryInfo &trj) {
  nlohmann::json result;
  result["start"] = start;
  result["end"] = end;
  struct Visitor {
    nlohmann::json &result;

    void operator()(const mt_rrt::samples::Blended &arc) const {
      result["type"] = "blended";
      result["start"] = arc.start;
      result["end"] = arc.end;
      result["ray"] = arc.ray;
      result["center"] = arc.center;
      result["arc_begin"] = arc.arc_begin;
      result["arc_end"] = arc.arc_end;
    }

    void operator()(const mt_rrt::samples::TrivialLine &line) const {
      result["type"] = "trivial";
    }
  } visitor{result["info"]};
  std::visit(visitor, trj);
  return result;
}

static const std::string PYTHON_SCRIPT =
    mt_rrt::merge(TEST_FOLDER, "CartTrajectoryTests.py");

#endif

static constexpr float TOLL = 1e-3f;

bool almost_equal(float a, float b) { return std::abs(a - b) < TOLL; }

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

  SECTION("aligned on same line, but bad orientations") {
    const State start = State{0, 0, to_rad(45)};
    const State end = State{1.f, 1.f, to_rad(60)};
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
class TrajTestBase {
public:
  const mt_rrt::State &getStart() const { return start; };
  const mt_rrt::State &getEnd() const { return end; };

protected:
  TrajTestBase(float angle, float distance, bool positive_or_negative_test) {
    start = {0, -distance, mt_rrt::utils::PI_HALF};
    end = {distance * cosf(angle), distance * sinf(angle), angle};
    if (!positive_or_negative_test) {
      end[2] += mt_rrt::utils::PI;
    }
  }

  mt_rrt::State start;
  mt_rrt::State end;
};

float to_0_2PI(float angle) {
  float result = angle;
  if (result < 0) {
    result += 2.f * mt_rrt::utils::PI;
  }
  return result;
}

class TrajTest : public TrajTestBase {
public:
  TrajTest(float angle, float distance,
           const mt_rrt::samples::CartSteerLimits &limits)
      : TrajTestBase(angle, distance, true) {
    expected_result = std::make_unique<mt_rrt::samples::Blended>(
        mt_rrt::samples::Blended{start, end});
    auto &info = *expected_result.get();

    const auto gamma_dir = mt_rrt::utils::Versor(
        mt_rrt::utils::Point{0, 0},
        mt_rrt::utils::diff(
            mt_rrt::utils::Versor{end[2]}.asPoint(),
            mt_rrt::utils::Versor{mt_rrt::utils::PI_HALF}.asPoint()));
    const auto gamma = gamma_dir.angle();

    const float angle_delta =
        gamma_dir.angleBetween(mt_rrt::utils::Versor{-mt_rrt::utils::PI_HALF});
    const float angle_delta_tan = std::abs(tanf(angle_delta));
    info.ray = distance * angle_delta_tan;
    info.ray = std::min(info.ray, limits.maxRadius());
    const auto diagonal = std::abs(info.ray / sinf(angle_delta));
    info.center = {diagonal * cosf(gamma), diagonal * sinf(gamma)};
    const auto d = info.ray / angle_delta_tan;
    info.arc_begin = {0, -d};
    info.arc_end = {d * cosf(angle), d * sinf(angle)};
  }

  bool check(const mt_rrt::samples::Blended &computed_pieces) const {
    return almost_equal(computed_pieces.start, expected_result->start) &&
           almost_equal(computed_pieces.end, expected_result->end) &&
           almost_equal(computed_pieces.ray, expected_result->ray) &&
           almost_equal(computed_pieces.center, expected_result->center) &&
           almost_equal(computed_pieces.arc_begin,
                        expected_result->arc_begin) &&
           almost_equal(computed_pieces.arc_end, expected_result->arc_end);
  }

  const mt_rrt::samples::Blended &getInfo() const { return *expected_result; }

private:
  std::unique_ptr<mt_rrt::samples::Blended> expected_result;
};

class TrajTestNegative : public TrajTestBase {
public:
  TrajTestNegative(float angle, float distance)
      : TrajTestBase(angle, distance, false) {}
};
} // namespace

TEST_CASE("Blended arc logics", mt_rrt::merge(TEST_TAG, "[cart_trajectory]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  CartSteerLimits limits(0.2f, 1.f);

  auto angle = GENERATE(to_rad(0), to_rad(-45.f), to_rad(45.f), to_rad(135.f),
                        to_rad(-135.f));

  SECTION("") {
    auto distance = GENERATE(0.5f, 2.f);

    SECTION("impossible") {
      TrajTestNegative test_case(angle, distance);
      auto traj = compute_cart_trajectory_info(test_case.getStart(),
                                               test_case.getEnd(), limits);
      CHECK_FALSE(traj);
    }

    SECTION("possible") {
      TrajTest test_case(angle, distance, limits);
      const auto &start = test_case.getStart();
      const auto &end = test_case.getEnd();

      auto traj = compute_cart_trajectory_info(start, end, limits);
      REQUIRE(traj);
      CHECK(std::holds_alternative<Blended>(traj.value()));
      auto &as_arc = std::get<Blended>(traj.value());

#ifdef TEST_LOGGING
      std::stringstream log_name;
      log_name << "blended_" << to_grad(angle);
      nlohmann::json log_json = log_case(start, end, traj.value());
      Logger::log(log_name.str(), log_json, PYTHON_SCRIPT);
#endif

      CHECK(test_case.check(as_arc));
    }
  }
}

namespace {
std::vector<mt_rrt::State> extract_curve(mt_rrt::TrajectoryPtr trj) {
  std::vector<mt_rrt::State> states;
  mt_rrt::AdvanceInfo advance_info = mt_rrt::AdvanceInfo::advanced;
  while (advance_info == mt_rrt::AdvanceInfo::advanced) {
    states.emplace_back(trj->getState());
    advance_info = trj->advance();
  }
  return states;
}

#ifdef TEST_LOGGING
nlohmann::json log_case(const mt_rrt::State &start, const mt_rrt::State &end,
                        const std::vector<mt_rrt::State> &sequence) {
  nlohmann::json result;
  result["start"] = start;
  result["end"] = end;

  result["sequence"] = sequence;

  return result;
}
#endif
} // namespace

TEST_CASE("Blended arc advance interpolation",
          mt_rrt::merge(TEST_TAG, "[cart_trajectory]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  CartSteerLimits limits(0.2f, 1.f);
  Cart cart(1.f, 1.f, limits);
  CartPosesConnector traj_factory(Scene{std::move(cart), {}});

  auto angle = GENERATE(to_rad(0), to_rad(-45.f), to_rad(45.f), to_rad(135.f),
                        to_rad(-135.f));

  auto dist = GENERATE(0.5f, 2.f);

  TrajTest test_case(angle, dist, limits);
  const auto &start = test_case.getStart();
  const auto &end = test_case.getEnd();

  auto traj = traj_factory.getTrajectory(start, end);
  REQUIRE(traj);

  auto traj_curve = extract_curve(std::move(traj));

#ifdef TEST_LOGGING
  std::stringstream log_name;
  log_name << "interpolated_" << to_grad(angle);
  nlohmann::json log_json = log_case(start, end, traj_curve);
  Logger::log(log_name.str(), log_json, PYTHON_SCRIPT);
#endif

  // check the generated curve
  REQUIRE(traj_curve.size() >= 3);
  CHECK(almost_equal(to_point(traj_curve.front()), to_point(start)));
  CHECK(almost_equal(to_0_2PI(traj_curve.front()[2]), to_0_2PI(start[2])));
  // CHECK(almost_equal(to_point(traj_curve.back()), to_point(end))); // not
  // true as the last state is not taken
  CHECK(almost_equal(to_0_2PI(traj_curve.back()[2]), to_0_2PI(end[2])));

  const auto &info = test_case.getInfo();

  auto lies_on_line = [](const mt_rrt::utils::Segment &line,
                         const mt_rrt::utils::Point &subject) {
    auto coeff = closest_on_line(subject, line);
    return distance(point_on_segment(coeff, line), subject) < 1e-3f;
  };

  using Iter = std::vector<State>::iterator;
  Iter arc_begin;
  {
    utils::Segment seg{to_point(info.start), info.arc_begin};
    arc_begin = std::find_if(traj_curve.begin(), traj_curve.end(),
                             [&seg, &lies_on_line](const State &s) {
                               return !lies_on_line(seg, to_point(s));
                             });
  }
  Iter arc_end;
  {
    utils::Segment seg{to_point(info.end), info.arc_end};
    arc_end = std::find_if(arc_begin, traj_curve.end(),
                           [&seg, &lies_on_line](const State &s) {
                             return lies_on_line(seg, to_point(s));
                           });
  }

  // lines are not checked as they are trivial

  auto CHECK_ARC = [&info](const Iter &start, const Iter &end) {
    std::for_each(start, end, [&info](const State &s) {
      REQUIRE(almost_equal(distance(info.center, to_point(s)), info.ray));
    });
    // TODO check also angle
  };
  CHECK_ARC(arc_begin, arc_end);
}
