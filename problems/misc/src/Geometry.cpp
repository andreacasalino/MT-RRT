/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/EuclidianConnector.h>

#include <Geometry.h>

#include <algorithm>

namespace mt_rrt::geom {
float dot_product(std::span<const float> a, std::span<const float> b) {
  float res = 0;
  for (int k = 0; k < a.size(); ++k) {
    res += a[k] * b[k];
  }
  return res;
}

float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

Point::Point(const float *data) : data_{data, 2} {}

PointAllocated::PointAllocated(float x, float y)
    : std::array<float, 2>{x, y}, Point{this->std::array<float, 2>::data()} {}

float distance(const Point &a, const Point &b) {
  return euclidean_distance(a.data(), b.data());
}

float dot_product(const Point &a, const Point &b) {
  return dot_product(a.data(), b.data());
}

PointAllocated sum(const Point &subject, const Point &to_add) {
  return sum(subject, to_add, 1.f);
}

PointAllocated sum(const Point &subject, const Point &to_add,
                   float to_add_scale) {
  float x = subject.x() + to_add_scale * to_add.x();
  float y = subject.y() + to_add_scale * to_add.y();
  return PointAllocated{x, y};
}

PointAllocated diff(const Point &subject, const Point &to_remove) {
  return sum(subject, to_remove, -1.f);
}

PointAllocated diff(const Point &subject, const Point &to_remove,
                    float to_remove_scale) {
  return sum(subject, to_remove, -to_remove_scale);
}

namespace {
std::span<const float> carve_state(std::size_t len,
                                   std::span<const float> buffer,
                                   std::size_t waypoint) {
  auto it_b = buffer.begin() + waypoint * len;
  auto it_e = it_b + len;
  return std::span<const float>{it_b, it_e};
}
} // namespace

float curve_length(const Solution &curve) {
  float res{0};
  auto [len, buffer] = curve.getRaw();
  std::size_t waypoints = buffer.size() / len;
  for (std::size_t w = 1; w < waypoints; ++w) {
    res += euclidean_distance(carve_state(len, buffer, w - 1),
                              carve_state(len, buffer, w));
  }
  return res;
}

namespace {
class Interpolator {
public:
  Interpolator(const Solution &curve) {
    auto [len, state] = curve.getRaw();
    len_ = len;
    state_ = state;

    std::size_t waypoints = state_.size() / len_;
    len_cumulated_.push_back(0);
    for (std::size_t w = 1; w < waypoints; ++w) {
      float dist = euclidean_distance(carve_state(len_, state_, w - 1),
                                      carve_state(len_, state_, w));
      float dist_cum = dist + len_cumulated_.back();
      len_cumulated_.push_back(dist_cum);
    }
  }

  void at(std::vector<float> &recipient, float normalized_scale) {
    float p = normalized_scale * len_cumulated_.back();

    auto it = std::upper_bound(len_cumulated_.begin(), len_cumulated_.end(), p);

    if (it == len_cumulated_.end()) {
      set(recipient, carve_state(len_, state_, len_cumulated_.size() - 1));
      return;
    }

    if (it == len_cumulated_.begin()) {
      set(recipient, {state_.data(), len_});
      return;
    }

    std::size_t w = static_cast<std::size_t>(it - len_cumulated_.begin());
    if (*it == p) {
      set(recipient, carve_state(len_, state_, w));
      return;
    }

    std::span<const float> waypoint_a = carve_state(len_, state_, w - 1);
    std::span<const float> waypoint_b = carve_state(len_, state_, w);

    float segment_dist = len_cumulated_[w] - len_cumulated_[w - 1];
    float scale = (p - len_cumulated_[w - 1]) / segment_dist;

    set(recipient, waypoint_a, waypoint_b, scale);
  }

private:
  void set(std::vector<float> &recipient, std::span<const float> giver) {
    recipient.clear();
    recipient.insert(recipient.end(), giver.begin(), giver.end());
  }

  void set(std::vector<float> &recipient, std::span<const float> giver_a,
           std::span<const float> giver_b, float scale) {
    float scale_complement = 1.f - scale;
    recipient.clear();
    for (int i = 0; i < giver_a.size(); ++i) {
      recipient.push_back(scale_complement * giver_a[i] + scale * giver_b[i]);
    }
  }

  std::size_t len_;
  std::span<const float> state_;
  std::vector<float> len_cumulated_;
};
} // namespace

float curve_similarity(const Solution &curve_a, const Solution &curve_b) {
  const float delta = 1.f / 100.f;
  Interpolator interp_a(curve_a);
  Interpolator interp_b(curve_b);
  std::size_t counter = 0;
  float result = 0;
  static thread_local std::vector<float> buffer_a, buffer_b;
  for (float s = 0.f; s < 1.f; s += delta, ++counter) {
    interp_a.at(buffer_a, s);
    interp_b.at(buffer_b, s);
    result += euclidean_distance(buffer_a, buffer_b);
  }
  return result / static_cast<float>(counter);
}
} // namespace mt_rrt::geom
