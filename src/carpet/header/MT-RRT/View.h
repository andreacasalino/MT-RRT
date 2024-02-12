/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <cstdlib>
#include <vector>

namespace mt_rrt {
struct View {
  View() = default;
  View(const float *data, std::size_t size) : size{size}, data{data} {}
  View(const std::vector<float> &owner) : View{owner.data(), owner.size()} {};

  View trim(std::size_t size, std::size_t from = 0) const {
    return View{data + from, size};
  }

  std::vector<float> convert() const;

  std::size_t size = 0;
  const float *data = nullptr;
};
} // namespace mt_rrt
