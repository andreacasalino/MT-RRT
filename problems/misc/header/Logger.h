/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <nlohmann/json.hpp>

#include <filesystem>
#include <unordered_set>

namespace mt_rrt {
class Logger {
public:
  static Logger &get();

  void add(const std::string &tag, const std::string &title,
           const nlohmann::json &content);

  const auto &tmpFolderPath() const { return tmpFolderPath_; }

private:
  // clean up the log folder when building a new singleton
  Logger();

  std::filesystem::path tmpFolderPath_;

  struct PairHash {
    std::size_t operator()(const std::pair<std::string, std::string> &p) const {
      static const std::hash<std::string> hasher;
      // Hash both elements
      std::size_t h1 = hasher(p.first);
      std::size_t h2 = hasher(p.second);
      // Combine the two hash values using a bit-mixing shift
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  std::unordered_set<std::pair<std::string, std::string>, PairHash> results;
};

std::string time_now();
} // namespace mt_rrt
