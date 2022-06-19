/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/Synchronization.h>

namespace mt_rrt {
SynchronizationDegree::SynchronizationDegree(const float initial_value)
    : Limited<float>(0, 0.5f, initial_value) {}

SynchronizationDegree::SynchronizationDegree()
    : Limited<float>(0, 0.5f, 0.1f) {}

} // namespace mt_rrt
