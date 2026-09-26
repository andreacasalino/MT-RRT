/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderBidir.h>

#include <MT-RRT/ExtenderUtils.h>

namespace mt_rrt {
std::vector<std::vector<float>> BidirSolution::getSequence() const {
  auto result = sequence_from_root(*byPassFront);
  auto result_to_append = sequence_from_root(*byPassBack);
  result.insert(result.end(), result_to_append.rbegin(),
                result_to_append.rend());
  return result;
}

float BidirSolution::cost() const {
  return byPassFront->cost2Root() + cost2Back + byPassBack->cost2Root();
}

ExtenderBidirectional::ExtenderBidirectional(TreeHandlerPtr front,
                                             TreeHandlerPtr back)
    : Extender(*front), front_handler{std::move(front)}, back_handler{
                                                             std::move(back)} {}

} // namespace mt_rrt
