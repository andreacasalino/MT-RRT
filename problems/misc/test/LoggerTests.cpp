#include <gtest/gtest.h>

#include <MT-RRT/Format.h>

#include <Logger.h>

#include <filesystem>
#include <unordered_map>

TEST(LoggerTest, check_the_logger) {

  ::testing::UnitTest::GetInstance()->current_test_info();

  std::string test_suite_name = test_info->test_suite_name();
  std::string test_name = test_info->name();

  std::vector<std::string> labels{};

  std::unordered_map<std::string, std::size_t> files{{"tag-a", 3},
                                                     {"tag-b", 2}};
  for (const auto &[tag, count] : files) {
    for (std::size_t k = 0; k < count; ++k) {
      mt_rrt::Logger::get().add(mt_rrt::format("{}-{}", k, k),
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
