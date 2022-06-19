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
/** @param the sampler to steal
 *  @param the trajectory factory to steal
 *  @param the dimension of the state space of the problem to solve. Refer
 * to 1.2
 *  @param the \gamma involved in the near set computation, refer to
 * Section 1.2.3 of the documentation
 *  @param true when the problem is simmetric.
 *  @throw if sampler or manager are nullptr
 */
struct ProblemDescription {
  SamplerPtr sampler;
  ConnectorPtr connector;
  bool simmetry;
  Positive<float> gamma;
};
using ProblemDescriptionPtr = std::shared_ptr<const ProblemDescription>;

ProblemDescriptionPtr
clone_description(const ProblemDescriptionPtr &description);

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
