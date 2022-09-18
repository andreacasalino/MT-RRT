#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

TEST_CASE("merge strings", TEST_TAG) {
  CHECK(mt_rrt::merge("first", "second") == "firstsecond");
  CHECK(mt_rrt::merge("first", "second", "third") == "firstsecondthird");

  CHECK(mt_rrt::merge("first", "second", "third", "again") ==
        "firstsecondthirdagain");
}

#include <MT-RRT-carpet/Error.h>

TEST_CASE("merged error message", TEST_TAG) {
  CHECK(mt_rrt::Error{"first"}.what() == std::string{"first"});
  CHECK(mt_rrt::Error{"first", "second"}.what() == std::string{"firstsecond"});
}
