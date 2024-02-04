#include <gtest/gtest.h>

#include <MT-RRT/Random.h>

#include <algorithm>
#include <memory>
#include <thread>
#include <unordered_set>

namespace {
template <typename Iter>
bool are_all_different(const Iter &begin, const Iter &end) {
  std::unordered_set<typename Iter::value_type> sorted{begin, end};
  return sorted.size() == std::distance(begin, end);
}

std::vector<float> sample(const mt_rrt::UniformEngine &engine) {
  std::vector<float> result;
  for (std::size_t k = 0; k < 10; ++k) {
    result.push_back(engine.sample());
  }
  return result;
}
} // namespace

TEST(RandomTest, random_seeds_generation) {
  std::vector<mt_rrt::Seed> seeds;
  for (std::size_t k = 0; k < 10; ++k) {
    seeds.push_back(mt_rrt::make_random_seed());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(are_all_different(seeds.begin(), seeds.end()));
}

class SamplingFromClonedGeneratorsTest : public ::testing::Test {
protected:
  void SetUp() override {
    engines.emplace_back(std::make_unique<mt_rrt::UniformEngine>(0, 1.f, 5));
    for (std::size_t k = 1; k < 4; ++k) {
      engines.emplace_back(
          std::make_unique<mt_rrt::UniformEngine>(*engines.back()));
    }
  }

  using EnginePtr = std::unique_ptr<mt_rrt::UniformEngine>;
  std::vector<EnginePtr> engines;

  void showEpics() const {
    std::cout
        << "--------------------------------------------------------------"
        << std::endl;
    for (const auto &epic : epics) {
      for (auto val : epic) {
        std::cout << ' ' << val;
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }

  bool allEpicsDifferent() const {
    for (std::size_t r = 0; r < epics.size(); ++r) {
      for (std::size_t c = r + 1; c < epics.size(); ++c) {
        if (epics[r] == epics[c]) {
          return false;
        }
      }
    }
    return true;
  }
  std::vector<std::vector<float>> epics;
};

TEST_F(SamplingFromClonedGeneratorsTest, serial_generation) {
  for (const auto &engine : this->engines) {
    epics.push_back(sample(*engine));
  }

  showEpics();
  EXPECT_TRUE(allEpicsDifferent());
}

#include <omp.h>

TEST_F(SamplingFromClonedGeneratorsTest, parallel_generation) {
  epics.resize(engines.size());
#pragma omp parallel num_threads(engines.size())
  {
    int th_id = omp_get_thread_num();
    epics[th_id] = sample(*engines[th_id]);
  }

  showEpics();
  EXPECT_TRUE(allEpicsDifferent());
}

namespace {
void add(std::vector<float> &recipient, const std::vector<float> &giver) {
  recipient.insert(recipient.end(), giver.begin(), giver.end());
}
} // namespace

using DeterminismTestFixture =
    ::testing::TestWithParam<std::pair<mt_rrt::Seed, bool>>;

TEST_P(DeterminismTestFixture, check_sampling_determinism) {
  auto epic_maker = [&]() {
    std::vector<float> result;
    std::unique_ptr<mt_rrt::UniformEngine> engine =
        std::make_unique<mt_rrt::UniformEngine>(0, 1.f, GetParam().first);
    result = sample(*engine);
    if (GetParam().second) {
      for (std::size_t e = 0; e < 2; ++e) {
        engine = std::make_unique<mt_rrt::UniformEngine>(*engine);
        add(result, sample(*engine));
      }
    }
    return result;
  };

  std::vector<std::vector<float>> epics;
  for (std::size_t k = 0; k < 5; ++k) {
    epics.push_back(epic_maker());
  }

  const auto &first_epic = epics.front();
  EXPECT_TRUE(are_all_different(first_epic.begin(), first_epic.end()));
  for (std::size_t k = 1; k < epics.size(); ++k) {
    EXPECT_EQ(epics[0], epics[k]);
  }
}

INSTANTIATE_TEST_SUITE_P(
    DeterminismTests, DeterminismTestFixture,
    ::testing::Values(std::make_pair(0, false), std::make_pair(10, false),
                      std::make_pair(100, false), std::make_pair(10000, false),
                      std::make_pair(0, true), std::make_pair(10, true),
                      std::make_pair(100, true), std::make_pair(10000, true)));
