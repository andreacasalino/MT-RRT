/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>

#include <Geometry.h>

namespace mt_rrt::geom {
float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

Point::Point(float x, float y) {
  auto &cont = container_.emplace();
  cont[0] = x;
  cont[1] = y;
  data_ = cont.data();
}

Point::Point(const View &v) {
  if (v.size != 2) {
    throw Error{"cannot convert View to Point as size is ", v.size};
  }
  data_ = v.data;
}

Point &Point::operator=(const Point &o) {
  Point tmp{o.data_[0], o.data_[1]};
  *this = std::move(tmp);
  return *this;
}

Point::Point(Point &&o) { *this = std::move(o); }

Point &Point::operator=(Point &&o) {
  container_.reset();
  if (o.container_.has_value()) {
    data_ = container_.emplace(std::move(o.container_.value())).data();
  } else {
    data_ = o.data_;
  }
  return *this;
}

namespace {
float distance_(const float *a, const float *b, std::size_t size) {
  float res = 0;
  for (std::size_t k = 0; k < size; ++k) {
    res += powf(a[k] - b[k], 2.f);
  }
  res = sqrtf(res);
  return res;
}
} // namespace

float distance(const View &a, const View &b) {
  return distance_(a.data, b.data, a.size);
}

float distance(const Point &a, const Point &b) {
  return distance_(a.data(), b.data(), 2);
}

namespace {
float dot_(const float *a, const float *b, std::size_t size) {
  float res = 0;
  for (std::size_t k = 0; k < size; ++k) {
    res += a[k] * b[k];
  }
  return res;
}
} // namespace

float dot(const View &a, const View &b) { return dot_(a.data, b.data, a.size); }

float dot(const Point &a, const Point &b) {
  return dot_(a.data(), b.data(), 2);
}

Point sum(const Point &subject, const Point &to_add,
          const std::optional<float> &to_add_scale) {
  float x = subject.data()[0];
  x +=
      to_add_scale ? to_add.data()[0] * to_add_scale.value() : to_add.data()[0];
  float y = subject.data()[1];
  y +=
      to_add_scale ? to_add.data()[1] * to_add_scale.value() : to_add.data()[1];
  return Point{x, y};
}

Point diff(const Point &subject, const Point &to_remove,
           const std::optional<float> &to_remove_scale) {
  float x = subject.data()[0];
  x -= to_remove_scale ? to_remove.data()[0] * to_remove_scale.value()
                       : to_remove.data()[0];
  float y = subject.data()[1];
  y -= to_remove_scale ? to_remove.data()[1] * to_remove_scale.value()
                       : to_remove.data()[1];
  return Point{x, y};
}

float curve_length(const std::vector<std::vector<float>> &sequence) {
  float result = 0;
  for (std::size_t k = 1; k < sequence.size(); ++k) {
    result += distance(sequence[k - 1], sequence[k]);
  }
  return result;
}

namespace {
class Interpolator {
public:
  Interpolator(const std::vector<std::vector<float>> &subject)
      : total_length(curve_length(subject)) {
    if (subject.size() < 2) {
      throw Error{"Invalid curve to interpolate"};
    }
    for (std::size_t k = 1; k < subject.size(); ++k) {
      const auto &start = subject[k - 1];
      const auto &end = subject[k];
      segments.emplace_back(
          Segment{View{start}, View{end}, distance(start, end)});
    }
    bufferForAt.resize(subject.front().size());
  }

  View at(const float s) {
    float length = s * total_length;
    float cumulated_length = 0;
    for (const auto &segment : segments) {
      if (length <= (cumulated_length + segment.length)) {
        const float b_coeff = (length - cumulated_length) / segment.length;
        const float a_coeff = 1.f - b_coeff;
        for (std::size_t k = 0; k < segment.end.size; ++k) {
          bufferForAt[k] =
              a_coeff * segment.start.data[k] + b_coeff * segment.end.data[k];
        }
        return View{bufferForAt};
      }
      cumulated_length += segment.length;
    }
    return segments.back().end;
  }

private:
  const float total_length;

  struct Segment {
    View start;
    View end;
    float length;
  };
  std::vector<Segment> segments;

  std::vector<float> bufferForAt;
};
} // namespace

float curve_similarity(const std::vector<std::vector<float>> &a,
                       const std::vector<std::vector<float>> &b) {
  const float delta = 1.f / 100.f;
  Interpolator interp_a(a);
  Interpolator interp_b(b);
  std::size_t counter = 0;
  float result = 0;
  for (float s = 0.f; s < 1.f; s += delta, ++counter) {
    result += distance(interp_a.at(s), interp_b.at(s));
  }
  return result / static_cast<float>(counter);
}
} // namespace mt_rrt::geom
