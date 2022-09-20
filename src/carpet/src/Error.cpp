/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Error.h>

namespace mt_rrt {
Error::Error(const std::string &what) : std::runtime_error(what) {}
} // namespace mt_rrt
