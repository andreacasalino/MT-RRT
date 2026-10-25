/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Limited.h>
#include <MT-RRT/Types.h>

#include <math.h>
#include <span>

namespace mt_rrt {
template <typename C>
concept TunneledConstraintsChecker = requires(C obj,
                                              std::span<const float> state) {
  { obj.check_at(state) } -> std::same_as<bool>;
};

template <typename C>
concept SegmentConstraintsChecker = requires(C obj, std::span<const float> from,
                                             std::span<const float> to) {
  { obj.check_from_to(from, to) } -> std::same_as<bool>;
};

template <typename C>
concept IsConstraintsChecker =
    (TunneledConstraintsChecker<C> ||
     SegmentConstraintsChecker<C>)&&std::is_base_of_v<Copiable<C>, C>;

[[nodiscard]] float euclidean_distance(std::span<const float> a,
                                       std::span<const float> b);

template <IsConstraintsChecker ConstraintsChecker>
class EuclidianConnector
    : public Copiable<EuclidianConnector<ConstraintsChecker>> {
public:
  EuclidianConnector(std::unique_ptr<ConstraintsChecker> checker,
                     Positive quantized_advancement, SteerIterations steers)
      : data_{quantized_advancement, steers, std::move(checker)} {}

  // euclidean distance in the state space
  Positive minCost2Go(std::span<const float> start,
                      std::span<const float> target) {
    return {euclidean_distance(start, target)};
  }

  std::optional<TraverseResult> steer(std::span<const float> start,
                                      std::span<const float> target,
                                      std::vector<float> &reached);

  std::unique_ptr<EuclidianConnector<ConstraintsChecker>> copy() const final {
    return std::make_unique<EuclidianConnector<ConstraintsChecker>>(
        data_.checker->copy(), data_.quantized_advancement, data_.steers);
  }

  struct Data {
    Positive quantized_advancement;
    SteerIterations steers;
    std::unique_ptr<ConstraintsChecker> checker;
  };

  const Data &get() const { return data_; }

  const ConstraintsChecker &getChecker() const { return *data_.checker; }

private:
  [[nodiscard]] bool canAdvance_(std::span<const float> current,
                                 std::span<const float> next) {
    if constexpr (SegmentConstraintsChecker<ConstraintsChecker>) {
      return data_.checker->check_from_to(current, next);
    } else {
      return data_.checker->check_at(next);
    }
  }

  Data data_;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <IsConstraintsChecker ConstraintsChecker>
std::optional<TraverseResult>
EuclidianConnector<ConstraintsChecker>::steer(std::span<const float> start,
                                              std::span<const float> target,
                                              std::vector<float> &reached) {
  float distance_tot = euclidean_distance(start, target);
  float distance_done{0};
  static thread_local std::vector<float> current;
  if constexpr (SegmentConstraintsChecker<ConstraintsChecker>) {
    current.clear();
    current.insert(current.end(), start.begin(), start.end());
  }
  for (std::size_t i{0}; i < data_.steers.get(); ++i) {
    if (std::abs(distance_tot - distance_done) <= data_.quantized_advancement) {
      if constexpr (TunneledConstraintsChecker<ConstraintsChecker>) {
        return TraverseResult{.target_was_reached = true,
                              .cost2Go = Positive{distance_tot}};
      } else {
        if (data_.checker->check_from_to(previous, reached)) {
          return TraverseResult{.target_was_reached = true,
                                .cost2Go = Positive{distance_tot}};
        }
        break;
      }
    }

    distance_done += data_.quantized_advancement.get();

    float scale = distance_done / distance_tot;
    float scale_complement = 1.f - scale;
    reached.clear();
    for (int i = 0; i < start.size(); ++i) {
      reached.push_back(scale_complement * start[i] + scale * target[i]);
    }

    bool can_advance = canAdvance_(current, reached);
    if (!can_advance) {
      if (i == 0) {
        distance_done = 0;
      } else {
        distance_done -= data_.quantized_advancement.get();
      }
      break;
    }

    std::swap(current, reached);
  }

  if (distance_done == 0) {
    return std::nullopt;
  } else {
    std::swap(current, reached);
    return TraverseResult{.target_was_reached = false,
                          .cost2Go = Positive{distance_done}};
  }
}
} // namespace mt_rrt
