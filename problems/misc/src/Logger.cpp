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
template<char ... Symbols>
struct Match {
  static bool match(char c) {
    return match_<Symbols...>(c);
  }

private:
  template<char first, char second, char ... Remaining>
  static bool match_(char c) {
    return first == c || 
           match_<second, Remaining...>(c);
  }
  
  template<char last>
  static bool match_(char c) {
    return last == c;
  }
};

template<char Replacer, char ... Symbols>
void replace(std::string& subject) {
  for(std::size_t k=0; k<subject.size(); ++k) {
    if(Match<Symbols...>::match(subject[k])) {
      subject[k] = Replacer;
    }
  }
}
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
  replace<'_', ' ', ':'>(res);
  return res;
}

Logger::Logger() {
  std::string prefix{"MT_RRT_"};

  // clean up the folder 
  std::vector<std::filesystem::path> oldResults;
  for(const auto& el : std::filesystem::directory_iterator(std::filesystem::temp_directory_path())) {
    const auto& p = el.path();
    if(std::filesystem::is_directory(p) && p.string().find({prefix}) == 0) {
      oldResults.emplace_back(p);
    }
  }
  for(const auto& p : oldResults) {
    std::filesystem::remove_all(p);
  }

  std::string name = prefix + time_now();
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
