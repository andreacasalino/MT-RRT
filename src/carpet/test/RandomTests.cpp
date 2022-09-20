#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Random.h>

#include <algorithm>
#include <memory>
#include <set>
#include <thread>

namespace {
template <typename Iter>
bool are_all_different(const Iter &begin, const Iter &end) {
  std::set<typename Iter::value_type> sorted;
  sorted.insert(begin, end);
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

TEST_CASE("random seeds generation", TEST_TAG) {
  std::vector<mt_rrt::Seed> seeds;
  for (std::size_t k = 0; k < 10; ++k) {
    seeds.push_back(mt_rrt::make_random_seed());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  CHECK(are_all_different(seeds.begin(), seeds.end()));
}

#include <omp.h>

TEST_CASE("sampling from cloned generators", TEST_TAG) {
  using EnginePtr = std::unique_ptr<mt_rrt::UniformEngine>;
  std::vector<EnginePtr> engines;

  engines.emplace_back(std::make_unique<mt_rrt::UniformEngine>(0, 1.f, 5));
  for (std::size_t k = 1; k < 4; ++k) {
    engines.emplace_back(
        std::make_unique<mt_rrt::UniformEngine>(*engines.back()));
  }

  auto parallel_generation = GENERATE(true, false);
  std::vector<std::vector<float>> epics;

  if (parallel_generation) {
    epics.resize(engines.size());
#pragma omp parallel num_threads(engines.size())
    {
      int th_id = omp_get_thread_num();
      epics[th_id] = sample(*engines[th_id]);
    }
  } else {
    for (const auto &engine : engines) {
      epics.push_back(sample(*engine));
    }
  }

  // check all epics are different from each other
  for (std::size_t r = 0; r < epics.size(); ++r) {
    for (std::size_t c = r + 1; c < epics.size(); ++c) {
      CHECK_FALSE(epics[r] == epics[c]);
    }
  }
}

namespace {
void add(std::vector<float> &recipient, const std::vector<float> &giver) {
  recipient.insert(recipient.end(), giver.begin(), giver.end());
}
} // namespace

TEST_CASE("check sampling determinism", TEST_TAG) {
  const mt_rrt::Seed seed = GENERATE(0, 10, 100, 10000);

  auto multi_generation = GENERATE(true, false);

  auto epic_maker = [&]() {
    std::vector<float> result;
    std::unique_ptr<mt_rrt::UniformEngine> engine =
        std::make_unique<mt_rrt::UniformEngine>(0, 1.f, seed);
    result = sample(*engine);
    if (multi_generation) {
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
  CHECK(are_all_different(first_epic.begin(), first_epic.end()));
  for (std::size_t k = 1; k < epics.size(); ++k) {
    CHECK(epics[0] == epics[k]);
  }
}
