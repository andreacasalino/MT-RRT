/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>
#include <MT-RRT/Planner.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/TreeBase.h>

namespace mt_rrt {
/**
 * @brief the classical mon-thread solver described in Section "Background on
 * RRT" of the documentation.
 */
template <IsProblemDescription P> class StandardPlanner {
  using ExtenderSingle = ExtenderSingle<P, TreeBase>;
  using ExtenderBidirectional = ExtenderBidirectional<P, TreeBase>;

public:
  StandardPlanner(Problem<P> problem) : problem_{std::move(problem)} {}

  std::pair<std::size_t, std::optional<Solution>>
  solve(PlannerSolution &recipient, std::span<const float> start,
        std::span<const float> end) {
    if constexpr (P::kExpansionStrategy == ExpansionStrategy::Single ||
                  P::kExpansionStrategy == ExpansionStrategy::Star) {
      return solve_(ExtenderSingle extender{
          end, TreeBase{start}, *problem_.connector, *problem_.sampler});
    } else if constexpr (P::kExpansionStrategy == ExpansionStrategy::Bidir) {
      return solve_(ExtenderBidirectional extender{
          TreeBase{start}, TreeBase{end}, *problem_.connector,
          *problem_.sampler});
    }
  }

protected:
  template <typename E>
  std::pair<std::size_t, std::optional<Solution>> solve_(E extender) {
    std::pair<std::size_t, std::optional<Solution>> res;

    res.first =
        extend_many(extender, std::make_shared<KeepSearchPredicate>(...));
    res.second = extender.materializeBestSolution();

    if (problem_.provide_extra_info) {
      auto &ref = recipient.extra_info.emplace();

      if constexpr (std::is_same_v<E, ExtenderSingle>) {
        ref.nodes.emplace_back(extender.tree_.extractNodes());
      } else {
        ref.nodes.emplace_back(extender.trees_.first.extractNodes());
        ref.nodes.emplace_back(extender.trees_.second.extractNodes());
      }

      for (const auto &sol : extender.getSolutions()) {
        ref.all_solutions.emplace_back(sol.materialize());
      }
    }

    return res;
  }

  Problem<P> problem_;
};
} // namespace mt_rrt
