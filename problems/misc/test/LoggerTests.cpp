#include <gtest/gtest.h>

#include <Logger.h>

#include <filesystem>
#include <unordered_map>

TEST(LoggerTest, check_the_logger) {
  std::unordered_map<std::string, std::size_t> files{{"tag-a", 3},
                                                     {"tag-b", 2}};
  for (const auto &[tag, count] : files) {
    for (std::size_t k = 0; k < count; ++k) {
      mt_rrt::Logger::get().add(tag, "result" + std::to_string(k),
                                nlohmann::json{});
    }
  }

  for (const auto &[tag, count_expected] : files) {
    ASSERT_TRUE(
        std::filesystem::exists(mt_rrt::Logger::get().tmpFolderPath() / tag));
    std::size_t count = 0;
    for (auto _ : std::filesystem::directory_iterator{
             mt_rrt::Logger::get().tmpFolderPath() / tag}) {
      ++count;
    }
    EXPECT_EQ(count, count_expected);
  }
}
