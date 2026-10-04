#include <gtest/gtest.h>

#include <MT-RRT/Node.h>
#include <MT-RRT/Nodes.h>

using namespace mt_rrt;

TEST(NodeTest, nodes_creation) {
  Nodes allocator;
  const Node *root{nullptr};

  {
    root = &allocator.push(std::vector<float>{0, 1.f, 2.f});
    EXPECT_FALSE(root->data().parent);

    auto state = root->data().state;
    EXPECT_EQ(state.size(), 3);
    EXPECT_EQ(state[0], 0);
    EXPECT_EQ(state[1], 1.f);
    EXPECT_EQ(state[2], 2.f);
    EXPECT_EQ(root->data().cost2Go.get(), 0.f);
    EXPECT_EQ(root->cost2Root(), 0.f);
  }

  {
    auto &added = allocator.push(std::vector<float>{0, -1.f, -2.f});

    added.setParent(*root, Positive{1.5f});

    auto state = root->data().state;
    EXPECT_EQ(state.size(), 3);
    EXPECT_EQ(added.data().cost2Go.get(), 1.5f);
    EXPECT_EQ(added.cost2Root(), 1.5f);
    EXPECT_EQ(added.data().parent, root);
  }
}

TEST(NodeTest, nodes_chain) {
  Nodes allocator;

  std::size_t S = 5;

  const Node *prev = nullptr;
  float buffer{0};
  for (std::size_t k = 0; k < S; ++k) {
    auto &added = allocator.push(std::span<const float>{&buffer, 1});
    if (k != 0) {
      added.setParent(*prev, 1.f);
    }
    prev = &added;
  }

  EXPECT_EQ(prev->cost2Root(), static_cast<float>(S - 1));
}
