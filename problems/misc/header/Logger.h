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

  void add(const std::string &label, const nlohmann::json &content) {
    this->add(std::string_view{label.data(), label.size()}, content);
  }

  // name label is deduced from gtest test_suite_name_name
  void add_test_result(const nlohmann::json &content);

  // name label is deduced from gtest test_suite_name_name_name_suffix
  void add_test_result(const nlohmann::json &content,
                       std::string_view name_suffix);

  static inline const std::filesystem::path LOG_PATH{MT_RRT_LOG_PATH};

private:
  // clean up the log folder when building a new singleton
  Logger();
};
} // namespace mt_rrt
