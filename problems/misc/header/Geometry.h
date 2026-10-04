/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>
#include <MT-RRT/Solution.h>

#ifdef _WIN32
#include <corecrt_math_defines.h>
#endif
#include <math.h>

#include <array>
#include <optional>
#include <span>
#include <variant>

namespace mt_rrt::geom {
[[nodiscard]] float dot_product(std::span<const float> a,
                                std::span<const float> b);

static constexpr float PI = static_cast<float>(M_PI);
static constexpr float PI_HALF = static_cast<float>(M_PI_2);

float to_rad(float angle);

float to_grad(float angle);

struct Point {
  explicit Point(const float *data);

  float x() const noexcept { return data_[0]; }
  float y() const noexcept { return data_[1]; }

  auto data() const { return data_; }

private:
  std::span<const float> data_;
};

struct PointAllocated : private std::array<float, 2>, Point {
  PointAllocated() : PointAllocated{0, 0} {}
  explicit PointAllocated(float x, float y);

  static PointAllocated clone(const Point &o) {
    return PointAllocated{o.data()[0], o.data()[1]};
  }

  PointAllocated(const PointAllocated &o) : PointAllocated{clone(o)} {}
  PointAllocated &operator=(const PointAllocated &o) {
    auto *data = this->std::array<float, 2>::data();
    data[0] = o.x();
    data[1] = o.y();
    return *this;
  }

  PointAllocated(PointAllocated &&o) noexcept : PointAllocated{clone(o)} {}
  PointAllocated &operator=(PointAllocated &&o) noexcept { return *this = o; }
};

float distance(const Point &a, const Point &b);

float dot_product(const Point &a, const Point &b);

[[nodiscard]] PointAllocated sum(const Point &subject, const Point &to_add);

[[nodiscard]] PointAllocated sum(const Point &subject, const Point &to_add,
                                 float to_add_scale);

[[nodiscard]] PointAllocated diff(const Point &subject, const Point &to_remove);

[[nodiscard]] PointAllocated diff(const Point &subject, const Point &to_remove,
                                  float to_remove_scale);

[[nodiscard]] float curve_length(const Solution &curve);

[[nodiscard]] float curve_similarity(const Solution &curve_a,
                                     const Solution &curve_b);
} // namespace mt_rrt::geom
