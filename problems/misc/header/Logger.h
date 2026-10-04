/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string_view>

namespace mt_rrt {
class Logger {
public:
  static Logger &get();

  void add(std::string_view label, const nlohmann::json &content);

  static inline const std::filesystem::path LOG_PATH{MT_RRT_LOG_PATH};

private:
  // clean up the log folder when building a new singleton
  Logger();
};
} // namespace mt_rrt
