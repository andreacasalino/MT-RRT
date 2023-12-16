#include <gtest/gtest.h>

#include <MT-RRT/Node.h>

using namespace mt_rrt;

TEST(NodeTest, nodes_creation) {
  NodesAllocator allocator;
  Node *root = nullptr;

  {
    root = &allocator.emplace_back(std::vector<float>{0, 1.f, 2.f});
    EXPECT_EQ(root->getParent(), nullptr);

    auto state = root->state();
    EXPECT_EQ(state.size, 3);
    EXPECT_EQ(state.data[0], 0);
    EXPECT_EQ(state.data[1], 1.f);
    EXPECT_EQ(state.data[2], 2.f);
    EXPECT_EQ(root->cost2Go(), 0.f);
    EXPECT_EQ(root->cost2Root(), 0.f);
  }

  {
    Node &added = allocator.emplace_back(std::vector<float>{0, -1.f, -2.f});

    EXPECT_EQ(added.state().size, 3);

    added.setParent(*root, 1.5f);
    EXPECT_EQ(added.cost2Go(), 1.5f);
    EXPECT_EQ(added.cost2Root(), 1.5f);
    EXPECT_EQ(added.getParent(), root);
  }
}

TEST(NodeTest, nodes_chain) {
  NodesAllocator tree;

  std::size_t S = 5;

  Node *prev = nullptr;
  for (std::size_t k = 0; k < S; ++k) {
    auto &added = tree.emplace_back(std::vector<float>{0});
    if (k != 0) {
      added.setParent(*prev, 1.f);
    }
    prev = &added;
  }

  EXPECT_EQ(prev->cost2Root(), static_cast<float>(S - 1));
}
