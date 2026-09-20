#include <gtest/gtest.h>

#include <MT-RRT/Format.h>

TEST(StringsTest, merge_strings) {
  auto formatted = mt_rrt::Format{"{} {}", "first", "second"}.to_string();
  EXPECT_EQ(formatted, "first second");

  // EXPECT_EQ(mt_rrt::merge("first", "second", "third", "again"),
  //           "firstsecondthirdagain");
}

#include <MT-RRT/Error.h>

TEST(StringsTest, compose_Error) {
  // EXPECT_EQ(mt_rrt::Error{"first"}.what(), std::string{"first"});

  // mt_rrt::Error err{"first", "second"};
  // EXPECT_EQ(err.what(), std::string{"firstsecond"});
}
