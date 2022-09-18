/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Limited.h>
#include <MT-RRT-core/Sampler.h>
#include <MT-RRT-core/Trajectory.h>

namespace mt_rrt {
/**
 * @brief Groups together all the static information characterizing the class of
 * problems to solve.
 */
struct ProblemDescription {
  SamplerPtr sampler;
  ConnectorPtr connector;
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
};
using ProblemDescriptionPtr = std::shared_ptr<const ProblemDescription>;

ProblemDescriptionPtr
clone_description(const ProblemDescriptionPtr &description);

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
