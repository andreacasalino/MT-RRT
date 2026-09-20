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
template <Connector C, Sampler S> struct ProblemDescription {
  /**
   * @brief when true, the path connecting start to end, can be traversed in the
   * opposite direction to connect end to start.
   */
  bool simmetry;
  /**
   * @brief \gamma involved in the near set computation, refer to
   * Section 1.2.3 of the documentation
   */
  Positive gamma;

  std::unique_ptr<S> sampler;
  std::unique_ptr<C> connector;
};

template <Connector C, Sampler S>
using ProblemDescriptionPtr = std::shared_ptr<const ProblemDescription<C, S>>;

template <Connector C, Sampler S> struct DescriptionAndParameters {
  ProblemDescriptionPtr<C, S> description;
  Parameters parameters;
};

/**
 * @brief Someone aware of the static description of the class of problems to
 * solve
 */
template <Connector C, Sampler S> class ProblemAware {
public:
  virtual ~ProblemAware() = default;

  ProblemAware(ProblemDescriptionPtr<C, S> description)
      : problem_{description} {}

  const auto &problem() const { return *problem_; };

private:
  ProblemDescriptionPtr<C, S> problem_;
};
} // namespace mt_rrt
