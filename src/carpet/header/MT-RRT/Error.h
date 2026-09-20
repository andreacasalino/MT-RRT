/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Format.h>
#include <stdexcept>

namespace mt_rrt {
class Error : public std::runtime_error {
public:
  explicit Error(std::string what);

  template <typename... Args>
  static Error make(std::string_view format, const Args &...args) {
    std::string msg = Format<Args...>{format, args...}.to_string();
    return Error{std::move(msg)};
  }
};
} // namespace mt_rrt
