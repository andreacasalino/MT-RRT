/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Types.h>

#include <memory>

namespace mt_rrt {

/**
 * @brief Groups together all the static information characterizing the class of
 * problems to solve.
 */
template <Connector C, Sampler S, bool Simmetry,
          ExpansionStrategy ExpansionStrategyT>
struct ProblemDescription {
  // TODO static assert if not Simmetry cannot have ExpansionStrategy star ore
  // bidir

  /**
   * @brief \gamma involved in the near set computation, refer to
   * Section 1.2.3 of the documentation
   */
  Positive gamma;

  using sampler_type = S;
  std::unique_ptr<S> sampler;

  using connector_type = C;
  std::unique_ptr<C> connector;

  /**
   * @brief when true, the path connecting start to end, can be traversed in the
   * opposite direction to connect end to start.
   */
  static inline constexpr bool kSimmetry = Simmetry;
  static inline constexpr ExpansionStrategy kExpansionStrategy =
      ExpansionStrategyT;
};

namespace details {
template <typename P> struct is_problem_description : std::true_type {};
// TODO fix me
// template <typename P> struct is_problem_description : std::false_type {};

// template <Connector C, Sampler S, bool Simmetry,
//           ExpansionStrategy ExpansionStrategyT>
// struct is_problem_description<
//     ProblemDescription<C, S, Simmetry, ExpansionStrategyT>> : std::true_type
//     {};
} // namespace details

template <typename P>
concept IsProblemDescription = details::is_problem_description<P>::value;

/**
 * @brief Groups together all the parameters that a @Planner neeeds to
 * know to solve a specific problem for connecting 2 pair of states.
 */
template <IsProblemDescription P>
using Problem = std::pair<P, ProblemParameters>;
} // namespace mt_rrt
