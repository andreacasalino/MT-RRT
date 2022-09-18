/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <sstream>
#include <string>

namespace mt_rrt {
namespace detail {
template <typename T> void merge(std::stringstream &recipient, const T &slice) {
  recipient << slice;
}

template <typename T, typename... Args>
void merge(std::stringstream &recipient, const T &slice,
           Args... slices_to_merge) {
  merge(recipient, slice);
  merge(recipient, std::forward<Args>(slices_to_merge)...);
}
} // namespace detail

/**
 * @brief put all the passed slices all together into a single string,
 * that is returned.
 */
template <typename T1, typename... Args>
std::string merge(const T1 &first, Args... slices_to_merge) {
  std::stringstream stream;
  detail::merge(stream, first, std::forward<Args>(slices_to_merge)...);
  return stream.str();
}
} // namespace mt_rrt
