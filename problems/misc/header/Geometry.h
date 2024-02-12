/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>
#include <MT-RRT/View.h>

#ifdef _WIN32
#include <corecrt_math_defines.h>
#endif
#include <math.h>

#include <array>
#include <optional>
#include <variant>

namespace mt_rrt::geom {
static constexpr float PI = static_cast<float>(M_PI);
static constexpr float PI_HALF = static_cast<float>(M_PI_2);

float to_rad(float angle);

float to_grad(float angle);

// important!!! b is assumed to have the same size of a,
// but no check is done inside this function
float distance(const View &a, const View &b);

// important!!! b is assumed to have the same size of a,
// but no check is done inside this function
float dot(const View &a, const View &b);

struct Point {
  Point() : Point{0, 0} {}
  Point(float x, float y);
  Point(const View &v);

  Point(const Point &o) : Point{o.data_[0], o.data_[1]} {}
  Point &operator=(const Point &o);

  Point(Point &&o);
  Point &operator=(Point &&o);

  const float *data() const { return data_; }

  View asView() const { return View{data_, 2}; }

  std::vector<float> asVec() const {
    return std::vector<float>{data_[0], data_[1]};
  }

private:
  std::optional<std::array<float, 2>> container_;
  const float *data_;
};

float distance(const Point &a, const Point &b);

float dot(const Point &a, const Point &b);

Point sum(const Point &subject, const Point &to_add,
          const std::optional<float> &to_add_scale = std::nullopt);

Point diff(const Point &subject, const Point &to_remove,
           const std::optional<float> &to_remove_scale = std::nullopt);

float curve_length(const std::vector<std::vector<float>> &sequence);

float curve_similarity(const std::vector<std::vector<float>> &a,
                       const std::vector<std::vector<float>> &b);
} // namespace mt_rrt::geom
