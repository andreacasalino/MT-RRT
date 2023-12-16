/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>

#include <Logger.h>

#include <fstream>

namespace mt_rrt {
Logger &Logger::get() {
  static Logger res = Logger{};
  return res;
}

Logger::Logger() {
  if (std::filesystem::exists(LOG_FOLDER)) {
    std::filesystem::remove_all(LOG_FOLDER);
  }
  std::filesystem::create_directories(LOG_FOLDER);
}

void Logger::add(const std::string &tag, const std::string &title,
                 const nlohmann::json &content) {
  std::filesystem::path p = LOG_FOLDER / tag;
  auto it = results.find(tag);
  if (it == results.end()) {
    std::filesystem::create_directories(p);
    it = results.emplace(tag, std::unordered_set<std::string>{}).first;
  }
  if (it->second.find(tag) != it->second.end()) {
    throw Error{merge(title, " was already used for topic ", tag)};
  }
  p /= title + ".json";
  std::ofstream stream{p};
  if (!stream.is_open()) {
    throw Error{"Can't open stream to ", p};
  }
  stream << content.dump(1);
}
} // namespace mt_rrt
