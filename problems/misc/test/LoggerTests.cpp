#include <gtest/gtest.h>

#include <MT-RRT/Format.h>

#include <Logger.h>

#include <filesystem>
#include <unordered_map>

#include <algorithm>
#include <ranges>

struct LoggerTest : ::testing::Test {
  void SetUp() override {
    std::sort(labels.begin(), labels.end());

    auto rng =
        labels | std::views::transform([](auto label) {
          return mt_rrt::Logger::LOG_PATH /
                 mt_rrt::format("LoggerTest_check_the_logger_{}.json", label);
        });
    expected_files = {rng.begin(), rng.end()};
  }

  void TearDown() override {
    std::vector<std::filesystem::path> files;
    for (const auto &entry :
         std::filesystem::directory_iterator{mt_rrt::Logger::LOG_PATH}) {
      files.emplace_back(entry);
    }
    std::sort(files.begin(), files.end());

    ASSERT_EQ(files, expected_files)
        << "Not the expected files left in the log directory by LoggerTest";
  }

  std::vector<std::string_view> labels{"tag-a", "tag-b", "tag-c"};
  std::vector<std::filesystem::path> expected_files;
};

TEST_F(LoggerTest, check_the_logger) {
  for (auto label : labels) {
    mt_rrt::Logger::get().add_test_result(nlohmann::json{label}, label);
  }
}
