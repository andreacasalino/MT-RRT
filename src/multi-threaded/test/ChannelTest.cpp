#include <gtest/gtest.h>

#include <MT-RRT/Channel.h>

#include <string>
#include <thread>

using namespace mt_rrt;

TEST(ChannelTest, push_while_poll) {
  Channel<std::string> c{20};

  // TODO
}
