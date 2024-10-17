/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/Single.h>

namespace mt_rrt::extender {
std::vector<std::vector<float>> SimpleSolution::materialize() const {
  auto result = sequence_from_root(*byPassNode);
  result.emplace_back(target.convert());
  return result;
}

} // namespace mt_rrt
