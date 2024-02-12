#include <gtest/gtest.h>

#include <NavigationProblem.h>
#include <NavigationProblemConversions.h>

using namespace mt_rrt;
using namespace mt_rrt::navigation;

namespace {
std::unordered_map<std::string, std::size_t> logs_count;

void log_case(const View &start, const View &end,
              const navigation::CartTrajectoryInfo &trj,
              const std::string &tag) {
  nlohmann::json result;
  result["start"] = start.convert();
  result["end"] = end.convert();
  struct Visitor {
    nlohmann::json &result;

    void operator()(const navigation::Blended &arc) const {
      result["type"] = "blended";
      result["start"] = arc.start.convert();
      result["end"] = arc.end.convert();
      result["ray"] = arc.ray;
      result["center"] = arc.center.asView().convert();
      result["arc_begin"] = arc.arc_begin.asView().convert();
      result["arc_end"] = arc.arc_end.asView().convert();
    }

    void operator()(const navigation::TrivialLine &line) const {
      result["type"] = "trivial";
    }
  } visitor{result};
  std::visit(visitor, trj);

  std::size_t count;
  if (auto it = logs_count.find(tag); it == logs_count.end()) {
    logs_count.emplace(tag, 1);
    count = 1;
  } else {
    count = ++it->second;
  }

  Logger::get().add(tag, "case-" + std::to_string(count), result);
}

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

bool almost_equal(const View &a, const View &b) {
  return almost_equal(a.data, b.data, 3);
}

bool almost_equal(const geom::Point &a, const geom::Point &b) {
  return almost_equal(a.data(), b.data(), 2);
}

using State = std::array<float, 3>;

View to_view(const State &state) { return View{state.data(), 3}; }
} // namespace

TEST(CartTrajectoryTest, trivial_lines) {
  navigation::CartSteerLimits limits;

  {
    SCOPED_TRACE("aligned on different lines");

    const State start = State{0, -0.5f, geom::to_rad(65)};
    const State end = State{3.f, 3.f, geom::to_rad(65)};
    auto traj =
        compute_cart_trajectory_info(to_view(start), to_view(end), limits);
    EXPECT_FALSE(traj);
  }

  {
    SCOPED_TRACE("aligned on same line, but opposite directions");

    const State start = State{0, 0, geom::to_rad(45.f)};
    const State end = State{1.f, 1.f, geom::to_rad(-135.f)};
    auto traj =
        compute_cart_trajectory_info(to_view(start), to_view(end), limits);
    EXPECT_FALSE(traj);
  }

  {
    SCOPED_TRACE("aligned on same line, but bad positions");

    const State start = State{1.f, 1.f, geom::to_rad(45)};
    const State end = State{0, 0, geom::to_rad(45)};
    auto traj =
        compute_cart_trajectory_info(to_view(start), to_view(end), limits);
    EXPECT_FALSE(traj);
  }

  {
    SCOPED_TRACE("aligned on same line, but bad orientations");

    const State start = State{0, 0, geom::to_rad(45)};
    const State end = State{1.f, 1.f, geom::to_rad(60)};
    auto traj =
        compute_cart_trajectory_info(to_view(start), to_view(end), limits);
    EXPECT_FALSE(traj);
  }

  {
    SCOPED_TRACE("alinged: trivial line");

    const State start = State{0, 0, geom::to_rad(45)};
    const State end = State{1.f, 1.f, geom::to_rad(45)};
    auto traj =
        compute_cart_trajectory_info(to_view(start), to_view(end), limits);
    ASSERT_TRUE(traj);
    ASSERT_TRUE(std::holds_alternative<TrivialLine>(traj.value()));
    auto &as_line = std::get<TrivialLine>(traj.value());

    log_case(to_view(start), to_view(end), traj.value(), "trivial_line");

    EXPECT_TRUE(almost_equal(as_line.start, to_view(start)));
    EXPECT_TRUE(almost_equal(as_line.end, to_view(end)));
  }
}

namespace {
template <bool PositiveOrNegativeTest> class TrajTestBase {
public:
  View getStart() const { return View{start.data(), 3}; };
  View getEnd() const { return View{end.data(), 3}; };

protected:
  TrajTestBase(float angle, float distance)
      : start{0, -distance, geom::PI_HALF}, end{distance * cosf(angle),
                                                distance * sinf(angle), angle} {
    if constexpr (!PositiveOrNegativeTest) {
      end[2] += geom::PI;
    }
  }

  State start;
  State end;
};

float to_0_2PI(float angle) {
  float result = angle;
  if (result < 0) {
    result += 2.f * geom::PI;
  }
  return result;
}

class TrajTest : public TrajTestBase<true> {
public:
  TrajTest(float angle, float distance,
           const navigation::CartSteerLimits &limits)
      : TrajTestBase<true>(angle, distance) {
    expected_result.emplace(navigation::Blended{getStart(), getEnd()});
    auto &info = expected_result.value();

    const auto gamma_dir = geom::Versor(
        geom::Point{0, 0}, geom::diff(geom::Versor{end[2]}.asPoint(),
                                      geom::Versor{geom::PI_HALF}.asPoint()));
    const auto gamma = gamma_dir.angle();

    const float angle_delta =
        gamma_dir.angleBetween(geom::Versor{-geom::PI_HALF});
    const float angle_delta_tan = std::abs(tanf(angle_delta));
    info.ray = distance * angle_delta_tan;
    info.ray = std::min(info.ray, limits.maxRadius());
    const auto diagonal = std::abs(info.ray / sinf(angle_delta));
    info.center = {diagonal * cosf(gamma), diagonal * sinf(gamma)};
    const auto d = info.ray / angle_delta_tan;
    info.arc_begin = {0, -d};
    info.arc_end = {d * cosf(angle), d * sinf(angle)};
  }

  bool check(const navigation::Blended &computed_pieces) const {
    return almost_equal(computed_pieces.start, expected_result->start) &&
           almost_equal(computed_pieces.end, expected_result->end) &&
           almost_equal(computed_pieces.ray, expected_result->ray) &&
           almost_equal(computed_pieces.center, expected_result->center) &&
           almost_equal(computed_pieces.arc_begin,
                        expected_result->arc_begin) &&
           almost_equal(computed_pieces.arc_end, expected_result->arc_end);
  }

  const navigation::Blended &getInfo() const { return *expected_result; }

private:
  std::optional<navigation::Blended> expected_result;
};

class TrajTestNegative : public TrajTestBase<false> {
public:
  TrajTestNegative(float angle, float distance)
      : TrajTestBase<false>(angle, distance) {}
};
} // namespace

TEST(CartTrajectoryTest, blended_arc_logic) {
  CartSteerLimits limits(0.2f, 1.f);

  for (float angle : std::vector<float>{geom::to_rad(0), geom::to_rad(-45.f),
                                        geom::to_rad(45.f), geom::to_rad(135.f),
                                        geom::to_rad(-135.f)}) {
    for (float distance : std::vector<float>{0.5f, 2.f}) {
      {
        SCOPED_TRACE("impossible");

        TrajTestNegative test_case(angle, distance);
        auto traj = compute_cart_trajectory_info(test_case.getStart(),
                                                 test_case.getEnd(), limits);
        EXPECT_FALSE(traj);
      }

      {
        SCOPED_TRACE("possible");

        TrajTest test_case(angle, distance, limits);
        const auto &start = test_case.getStart();
        const auto &end = test_case.getEnd();

        auto traj = compute_cart_trajectory_info(start, end, limits);
        ASSERT_TRUE(traj);
        ASSERT_TRUE(std::holds_alternative<Blended>(traj.value()));
        auto &as_arc = std::get<Blended>(traj.value());

        log_case(test_case.getStart(), test_case.getEnd(), *traj,
                 "blended_" + std::to_string(geom::to_grad(angle)));

        EXPECT_TRUE(test_case.check(as_arc));
      }
    }
  }
}

namespace {
std::vector<std::vector<float>> extract_curve(mt_rrt::TrajectoryPtr trj) {
  std::vector<std::vector<float>> states;
  Trajectory::AdvanceInfo advance_info = Trajectory::AdvanceInfo::advanced;
  while (advance_info == Trajectory::AdvanceInfo::advanced) {
    states.emplace_back(trj->getState().convert());
    advance_info = trj->advance();
  }
  return states;
}

bool lies_on_line(const geom::Segment &line, const geom::Point &subject) {
  auto coeff = line.closest_on_line(subject);
  return distance(line.at(coeff), subject) < 1e-3f;
};

} // namespace

TEST(CartTrajectoryTest, blended_arc_advance_interpolation) {
  CartSteerLimits limits(0.2f, 1.f);
  Cart cart(1.f, 1.f, limits);
  CartPosesConnector traj_factory(Scene{std::move(cart), {}});

  for (float angle : std::vector<float>{geom::to_rad(0), geom::to_rad(-45.f),
                                        geom::to_rad(45.f), geom::to_rad(135.f),
                                        geom::to_rad(-135.f)}) {
    for (float dist : std::vector<float>{0.5f, 2.f}) {
      TrajTest test_case(angle, dist, limits);
      const auto &start = test_case.getStart();
      const auto &end = test_case.getEnd();

      auto traj = traj_factory.getTrajectory(start, end);
      ASSERT_TRUE(traj);

      auto traj_curve = extract_curve(std::move(traj));

      {
        nlohmann::json result;
        result["start"] = start;
        result["end"] = end;
        result["sequence"] = traj_curve;
        std::string title = "case_" + std::to_string(geom::to_grad(angle)) +
                            "_dist_" + std::to_string(dist);
        Logger::get().add("interpolated", title, result);
      }

      {
        SCOPED_TRACE("check waypoints");

        ASSERT_TRUE(traj_curve.size() >= 3);
        EXPECT_TRUE(almost_equal(traj_curve.front(), start.convert()));
        EXPECT_TRUE(almost_equal(to_0_2PI(traj_curve.front()[2]),
                                 to_0_2PI(start.data[2])));
        EXPECT_TRUE(almost_equal(to_0_2PI(traj_curve.back()[2]),
                                 to_0_2PI(end.data[2])));
      }

      const auto &info = test_case.getInfo();

      {
        SCOPED_TRACE("check the interpolation along the arc piece");

        using Iter = std::vector<std::vector<float>>::const_iterator;
        Iter arc_begin;
        {
          geom::Segment seg{geom::Point{info.start.trim(2)}, info.arc_begin};
          arc_begin = std::find_if(
              traj_curve.begin(), traj_curve.end(), [&seg](const auto &s) {
                return !lies_on_line(seg, geom::Point{s[0], s[1]});
              });
        }
        Iter arc_end;
        {
          geom::Segment seg{geom::Point(info.end.trim(2)), info.arc_end};
          arc_end =
              std::find_if(arc_begin, traj_curve.cend(), [&seg](const auto &s) {
                return lies_on_line(seg, geom::Point{s[0], s[1]});
              });
        }

        // check allpoints are at the ray distance

        std::for_each(arc_begin, arc_end, [&info](const auto &s) {
          float distance = geom::distance(info.center.asView(), View{s});
          EXPECT_TRUE(almost_equal(distance, info.ray));
        });

        // TODO check also angle
      }
    }
  }
}
