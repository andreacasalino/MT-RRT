/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Rewiring.h>

namespace mt_rrt {
Rewiring::Rewiring(Positive gamma, std::size_t state_space_size)
    : gamma_{gamma}, state_space_size_{state_space_size} {}
} // namespace mt_rrt
