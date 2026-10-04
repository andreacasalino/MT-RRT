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
    TunneledConstraintsChecker<C> || SegmentConstraintsChecker<C>;

[[nodiscard]] float euclidean_distance(std::span<const float> a,
                                       std::span<const float> b);

template <IsConstraintsChecker ConstraintsChecker> class EuclidianConnector {
public:
  EuclidianConnector(Positive quantized_advancement, SteerIterations steers,
                     std::unique_ptr<ConstraintsChecker> checker)
      : data_{quantized_advancement, steers, std::move(checker)} {}

  // euclidean distance in the state space
  Positive minCost2Go(std::span<const float> start,
                      std::span<const float> target) {
    return {euclidean_distance(start, target)};
  }

  std::optional<TraverseResult> steer(std::span<const float> start,
                                      std::span<const float> target,
                                      std::vector<float> &reached);

  struct Data {
    Positive quantized_advancement;
    SteerIterations steers;
    std::unique_ptr<ConstraintsChecker> checker;
  };

  const auto &get() const { return data_; }

private:
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
  static thread_local std::vector<float> previous;
  if constexpr (SegmentConstraintsChecker<ConstraintsChecker>) {
    previous.clear();
    previous.insert(previous.end(), start.begin(), start.end());
  }
  for (std::size_t i{0}; i < data_.steers.get(); ++i) {
    if (distance_tot - distance_done < data_.quantized_advancement.get()) {
      reached.clear();
      reached.insert(reached.end(), target.begin(), target.end());
      return TraverseResult{.target_was_reached = true,
                            .cost2Go = Positive{distance_tot}};
    }

    distance_done += data_.quantized_advancement.get();

    float scale = distance_done / distance_tot;
    float scale_complement = 1.f - scale;
    reached.clear();
    for (int i = 0; i < start.size(); ++i) {
      reached.push_back(scale_complement * start[i] + scale * target[i]);
    }

    if constexpr (SegmentConstraintsChecker<ConstraintsChecker>) {
      if (!data_.checker->check_from_to(previous, reached)) {
        // back to previous
        std::swap(previous, reached);
        if (i == 0) {
          distance_done = 0;
        } else {
          distance_done -= data_.quantized_advancement.get();
        }
        break;
      }
      std::swap(previous, reached);
    } else {
      if (!data_.checker->check_at(reached)) {
        break;
      }
    }
  }

  if (distance_done == 0) {
    return std::nullopt;
  } else {
    return TraverseResult{.target_was_reached = false,
                          .cost2Go = Positive{distance_done}};
  }
}
} // namespace mt_rrt
