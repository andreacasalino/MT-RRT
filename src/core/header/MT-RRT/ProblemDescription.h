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
template <typename P>
concept ProblemDescription = requires(S obj) {
  { obj_const.gamma } -> std::same_as<Positive>;

  requires Sampler<typename P::sampler_type>;
  { obj.sampler } -> std::same_as<std::unique_ptr<typename P::sampler_type>>;

  requires Connector<typename P::connector_type>;
  {
    obj.connector
    } -> std::same_as<std::unique_ptr<typename P::connector_type>>;

  { P::simmetry_value } -> std::same_as<constexpr bool>;
};

/**
 * @brief Groups together all the static information characterizing the class of
 * problems to solve.
 */
template <Connector C, Sampler S, bool Simmetry>
struct ProblemDescriptionConcrete {
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
  static constexpr bool simmetry_value = Simmetry;
};
} // namespace mt_rrt
