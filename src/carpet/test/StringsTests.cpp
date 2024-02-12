#include <gtest/gtest.h>

#include <MT-RRT/Strings.h>

TEST(StringsTest, merge_strings) {
  EXPECT_EQ(mt_rrt::merge("first", "second"), "firstsecond");
  EXPECT_EQ(mt_rrt::merge("first", "second", "third"), "firstsecondthird");

  EXPECT_EQ(mt_rrt::merge("first", "second", "third", "again"),
            "firstsecondthirdagain");
}

#include <MT-RRT/Error.h>

TEST(StringsTest, compose_Error) {
  EXPECT_EQ(mt_rrt::Error{"first"}.what(), std::string{"first"});

  mt_rrt::Error err{"first", "second"};
  EXPECT_EQ(err.what(), std::string{"firstsecond"});
}
