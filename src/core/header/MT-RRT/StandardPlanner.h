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
public:
  StandardPlanner(Problem<P> problem);

  std::pair<std::size_t, std::optional<Solution>>
  solve(PlannerSolution &recipient, std::span<const float> start,
        std::span<const float> end, Problem<P> &problem) {
    std::pair<std::size_t, std::optional<Solution>> res;

    using ExtenderSingle = ExtenderSingle<TreeBase, C, S>;
    using ExtenderBidirectional = ExtenderBidirectional<TreeBase, C, S>;

    auto solve_ = [&](auto extender) {
      res.first = extend_iterations(extender,
                                    std::make_shared<KeepSearchPredicate>(...));
      res.second = extender.materializeBestSolution();

      if (parameters.provide_extra_info) {
        auto &ref = recipient.extra_info.emplace();

        if constexpr (std::is_same_v<decltype(extender), ExtenderSingle>) {
          ref.nodes.emplace_back(extender.tree_.extractNodes());
        } else {
          ref.nodes.emplace_back(extender.trees_.first.extractNodes());
          ref.nodes.emplace_back(extender.trees_.second.extractNodes());
        }

        for (const auto &sol : extender.getSolutions()) {
          ref.all_solutions.emplace_back(sol.materialize());
        }
      }
    };

    switch (parameters.expansion_strategy) {
    case ExpansionStrategy::Single:
    case ExpansionStrategy::Star: {
      solve_(ExtenderSingle extender{end, TreeBase{start}, *problem_.connector,
                                     *problem_.sampler});
    } break;
    case ExpansionStrategy::Bidir: {
      solve_(ExtenderBidirectional extender{TreeBase{start}, TreeBase{end},
                                            *problem_.connector,
                                            *problem_.sampler});
    } break;
    }

    return res;
  }

protected:
  Problem<P> problem_;
};
} // namespace mt_rrt
