/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Strings.h>
#include <stdexcept>

namespace mt_rrt {
class Error : public std::runtime_error {
public:
  explicit Error(const std::string &what);

  template <typename T1, typename T2, typename... Args>
  Error(const T1 &first, const T2 &second, Args &&...slices_to_merge)
      : Error(merge(first, second, std::forward<Args>(slices_to_merge)...)) {}
};
} // namespace mt_rrt
