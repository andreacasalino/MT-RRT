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
 * @brief The kind of strategy to use, refer to documentation at
 * Sections 1.2.1, 1.2.2 and 1.2.3
 */
enum class ExpansionStrategy { Single, Bidir, Star };

/**
 * @brief Groups together all the static information characterizing the class of
 * problems to solve.
 */
template <Connector C, Sampler S, bool Simmetry,
          ExpansionStrategy ExpansionStrategyT>
struct ProblemDescription {
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
} // namespace mt_rrt
