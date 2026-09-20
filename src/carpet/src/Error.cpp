/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>

namespace mt_rrt {
Error::Error(std::string what) : std::runtime_error(std::move(what)) {}
} // namespace mt_rrt
