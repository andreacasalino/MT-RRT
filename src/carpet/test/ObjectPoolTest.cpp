#include <gtest/gtest.h>

#include <MT-RRT/ObjectPool.h>

#include <sstream>

class SomeType {
public:
  SomeType(const std::string &name, const std::string &surname) {
    std::stringstream buffer;
    buffer << name << '-' << surname;
    id = buffer.str();
  }

  SomeType(const SomeType &) = delete;
  SomeType &operator=(const SomeType &) = delete;
  SomeType(SomeType &&) = delete;
  SomeType &operator=(SomeType &&) = delete;

  const auto &getId() const { return id; }

private:
  std::string id;
};

TEST(ObjectPoolTest, complex_type_pool) {
  mt_rrt::ObjectPool<SomeType> pool;

  auto makeId = [](std::size_t k) {
    std::stringstream buffer;
    buffer << "name" << k << "-"
           << "surname" << k;
    return buffer.str();
  };

  std::vector<SomeType *> added;
  for (std::size_t k = 0; k < 3; ++k) {
    auto &a = pool.emplace_back("name" + std::to_string(k),
                                "surname" + std::to_string(k));
    added.push_back(&a);
  }
  for (std::size_t k = 0; k < added.size(); ++k) {
    EXPECT_EQ(added[k]->getId(), makeId(k));
  }
}

TEST(ObjectPoolTest, trivial_type_pool) {
  mt_rrt::ObjectPool<float> pool;

  {
    // add individually
    std::size_t s = 10;
    std::vector<float *> added;
    for (std::size_t k = 0; k < s; ++k) {
      float &a = pool.emplace_back(static_cast<float>(k));
      added.push_back(&a);
    }
    for (std::size_t k = 0; k < added.size(); ++k) {
      EXPECT_EQ(*added[k], static_cast<float>(k));
    }
  }

  {
    // add many
    std::vector<float> values;
    for (std::size_t k = 0; k < mt_rrt::ObjectPool<float>::INITIAL_CAPACITY;
         ++k) {
      values.push_back(static_cast<float>(k) + 15);
    }

    float *added = pool.emplace_back_multiple(values.data(), values.size());
    EXPECT_EQ(*added, values.front());
    added = &added[values.size() - 1];
    EXPECT_EQ(*added, values.back());
  }
}
