/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Copiable.h>
#include <MT-RRT/Random.h>

#include <vector>

namespace mt_rrt {
/**
 * @brief Interface for a sampler of states.
 */
template <typename S>
concept Sampler = requires(const S obj_const, std::vector<float> &recipient) {
  /**
   * @brief Returns a state randomly sampled in the \mathcal{X}
   * space, Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation.
   * This random state are used for randomly growing searching trees.
   * @return a drawn random state.
   */
  { obj_const.sampleState(recipient) } -> std::same_as<void>;

  /**
   * @return a random seed to use for intializing another Sampler.
   */
  { obj_const.sampleSeed() } -> std::same_as<Seed>;
}
&&std::is_base_of_v<Copiable<S>, S>;
} // namespace mt_rrt
