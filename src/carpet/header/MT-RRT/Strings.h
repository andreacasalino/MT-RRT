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
/**
 * @brief put all the passed slices all together into a single string,
 * that is returned.
 */
template <typename... Args> std::string merge(Args &&...slices_to_merge) {
  std::stringstream stream;
  auto pack = [&](auto &&element) { stream << element; };
  (pack(slices_to_merge), ...);
  return stream.str();
}
} // namespace mt_rrt
