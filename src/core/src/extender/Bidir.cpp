/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/Bidir.h>

#include <algorithm>

namespace mt_rrt::extender {
std::vector<std::vector<float>> BidirSolution::materialize() const {
  auto result = sequence_from_root(*byPassFront);
  auto result_to_append = sequence_from_root(*byPassBack);
  std::for_each(result_to_append.rbegin(), result_to_append.rend(),
                [&result](std::vector<float> &seq) {
                  result.emplace_back(std::move(seq));
                });
  return result;
}

} // namespace mt_rrt
