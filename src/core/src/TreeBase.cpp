/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeBase.h>

namespace mt_rrt {
TreeBase::TreeBase(std::span<const float> root) { nodes_.push(root); }
} // namespace mt_rrt
