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

namespace mt_rrt {

/**
 * @brief Groups together all the static information characterizing the class of
 * problems to solve.
 */
struct ProblemDescription {
  /**
   * @brief when true, the path connecting start to end, can be traversed in the
   * opposite direction to connect end to start.
   */
  bool simmetry;
  /**
   * @brief \gamma involved in the near set computation, refer to
   * Section 1.2.3 of the documentation
   */
  Positive<float> gamma;

  SamplerPtr sampler;
  ConnectorPtr connector;
};
using ProblemDescriptionPtr = std::shared_ptr<const ProblemDescription>;

struct DescriptionAndParameters {
  const ProblemDescription &description;
  const Parameters &parameters;
};

/**
 * @brief Someone aware of the static description of the class of problems to
 * solve
 */
class ProblemAware {
public:
  virtual ~ProblemAware() = default;

  ProblemAware(const ProblemDescriptionPtr &description);

  ProblemAware(const ProblemAware &o) : ProblemAware(o.problem_){};

  const ProblemDescription &problem() const { return *problem_; };

protected:
  ProblemDescriptionPtr problemPtr() const { return problem_; }

private:
  ProblemDescriptionPtr problem_;
};

} // namespace mt_rrt
