/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>

#include <Logger.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace mt_rrt {
Logger &Logger::get() {
  static Logger res = Logger{};
  return res;
}

namespace {
const std::unordered_set<char> TO_REPLACE = std::unordered_set<char>{' ', ':'};
}

std::string time_now() {
  std::string res;
  {
    std::time_t now =
        std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream stream;
    stream << std::put_time(std::gmtime(&now), "%c %Z");
    res = stream.str();
  }
  for (auto &c : res) {
    if (TO_REPLACE.find(c) != TO_REPLACE.end())
      c = '_';
  }
  return res;
}

Logger::Logger() {
  std::string name = "MT_RRT_" + time_now();
  tmpFolderPath_ = std::filesystem::temp_directory_path() / name;
  std::ofstream{MT_RRT_LOG_PATH} << tmpFolderPath_.string();
}

void Logger::add(const std::string &tag, const std::string &title,
                 const nlohmann::json &content) {
  std::filesystem::path p = tmpFolderPath_ / tag;
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
