/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <unordered_map>
#include <unordered_set>

namespace mt_rrt {
class Logger {
public:
  static Logger &get();

  void add(const std::string &tag, const std::string &title,
           const nlohmann::json &content);

  static const auto &logRoot() { return LOG_FOLDER; }

private:
  // clean up the log folder when building a new singleton
  Logger();

  static inline const std::filesystem::path LOG_FOLDER =
      std::filesystem::path{MT_RRT_LOG_FOLDER};

  std::unordered_map<std::string, std::unordered_set<std::string>> results;
};
} // namespace mt_rrt
