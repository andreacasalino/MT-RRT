/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/Format.h>

#include <Logger.h>

#include <fstream>

namespace mt_rrt {
Logger::Logger() {
  // nuke the log folder
  std::filesystem::remove_all(Logger::LOG_PATH);
  std::filesystem::create_directories(Logger::LOG_PATH);
}

Logger &Logger::get() {
  static Logger res{};
  return res;
}

void Logger::add(std::string_view label, const nlohmann::json &content) {
  auto destination = Logger::LOG_PATH / format("{}.json", label);
  std::ofstream stream{destination};
  if (!stream.is_open()) {
    throw Error::make("Can't open stream to: {}", destination);
  }
  stream << content.dump(1);
}
} // namespace mt_rrt
