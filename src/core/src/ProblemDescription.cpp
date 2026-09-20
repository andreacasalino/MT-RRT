/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/ProblemDescription.h>

namespace mt_rrt {
ProblemAware::ProblemAware(const ProblemDescriptionPtr &prbl) {
  if (nullptr == prbl) {
    throw Error{"Empty problem description"};
  }
  problem_ = prbl;
}
} // namespace mt_rrt
