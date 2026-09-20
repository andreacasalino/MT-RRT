#include <gtest/gtest.h>

#include <MT-RRT/Format.h>

TEST(FormatTest, merge_two_args) {
  auto formatted = mt_rrt::Format{"{} {}", "first", "second"}.to_string();
  EXPECT_EQ(formatted, "first second");
}

TEST(FormatTest, merge_multiple_args) {
  auto formatted =
      mt_rrt::Format{"{} {}{} {}", "first", "second", "third", "again"}
          .to_string();
  EXPECT_EQ(formatted, "first secondthird again");
}

#include <MT-RRT/Error.h>

TEST(StringsTest, compose_Error) {
  auto exc = mt_rrt::Error::make("Exception reasons: {} {}", "first", 2);
  EXPECT_EQ(exc.what(), std::string{"Exception reasons: first 2"});
}
