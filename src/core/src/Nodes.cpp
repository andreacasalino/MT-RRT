/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Nodes.h>

namespace mt_rrt {
Nodes::Nodes() : statesPool_{std::make_unique<ObjectPool<float>>()} {};
} // namespace mt_rrt
