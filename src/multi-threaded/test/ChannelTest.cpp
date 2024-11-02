#include <gtest/gtest.h>

#include <MT-RRT/Channel.h>

#include <array>
#include <string>
#include <thread>

using namespace mt_rrt;

class CircularSequence {
public:
  CircularSequence() {
    for (std::size_t k = 0; k < 10; ++k) {
      buffer[k] = std::string{"message-"} + std::to_string(k);
    }
  }

  const std::string &get() { return buffer[pos_ % buffer.size()]; }

  void next() { ++pos_; }

private:
  std::size_t pos_{0};
  std::array<std::string, 10> buffer;
};

bool always_true() { return true; }

template <typename StopPred>
std::vector<std::string> consume(Channel<std::string> &channel,
                                 StopPred stopPred) {
  std::vector<std::string> res;
  std::string str;
  while (true) {
    if (channel.poll(str)) {
      res.emplace_back(std::move(str));
    } else if (stopPred()) {
      break;
    }
  }
  return res;
}

struct ChannelTest : ::testing::Test {
  template <bool PollTillAllFlushed>
  void produce(std::size_t how_many, ProducerSideChannel<std::string> &channel) {
    produce_(how_many, &channel);
    if constexpr (PollTillAllFlushed) {
      while (!channel.pending.empty()) {
        channel.poll();
      }
    }
  }

  void produce(std::size_t how_many) { produce_(how_many, channel.get()); }

  template <typename StopPred>
  std::vector<std::string> consume(StopPred stopPred) {
    return ::consume(*channel, stopPred);
  }

  bool checkSequence(const std::vector<std::string> &seq) {
    return std::all_of(
        seq.begin(), seq.end(),
        [expectedSeq = CircularSequence{}](const std::string &str) mutable {
          bool res = str == expectedSeq.get();
          expectedSeq.next();
          return res;
        });
  }

  static inline constexpr std::size_t Capacity = 15;
  ChannelPtr<std::string> channel =
      std::make_shared<Channel<std::string>>(Capacity);

private:
  template <typename ChannelT>
  void produce_(std::size_t how_many, ChannelT *channel) {
    CircularSequence generator;
    for (std::size_t k = 0; k < how_many; ++k, generator.next()) {
      channel->push(generator.get());
    }
  }
};

TEST_F(ChannelTest, push_lessThanCapacity_then_poll) {
  std::size_t toProduce = Capacity - 5;
  produce(toProduce);
  auto polled = consume(&always_true);
  ASSERT_EQ(polled.size(), toProduce);
  ASSERT_TRUE(checkSequence(polled));
}

TEST_F(ChannelTest, push_equalToCapacity_then_poll) {
  std::size_t toProduce = Capacity;
  produce(toProduce);
  auto polled = consume(&always_true);
  ASSERT_EQ(polled.size(), toProduce);
  ASSERT_TRUE(checkSequence(polled));
}

TEST_F(ChannelTest, push_moreThanCapacity_then_poll) {
  std::size_t toProduce = Capacity + 5;
  produce(toProduce);
  auto polled = consume(&always_true);
  ASSERT_EQ(polled.size(), Capacity);
  ASSERT_TRUE(checkSequence(polled));
}

TEST_F(ChannelTest, push_while_poll) {
  std::vector<std::string> polled;

  std::atomic_bool done{false};
  std::thread consumer{[&]() {
    polled = this->consume([&done]() {
      return done.load(std::memory_order::memory_order_acquire);
    });
  }};
  ProducerSideChannel<std::string> out{channel};
  produce<true>(100, out);
  done.store(true, std::memory_order::memory_order_release);
  consumer.join();

  ASSERT_EQ(polled.size(), 100);
  ASSERT_TRUE(checkSequence(polled));
}

struct NetworkFixture : public Network<std::string> {
  using Network<std::string>::incoming;
};

struct NetworkTest : ::testing::Test {
  void SetUp() {
    network.resize(4);
    std::vector<std::reference_wrapper<Network<std::string>>> references;
    for (auto &ref : network) {
      references.emplace_back(ref);
    }
    Network<std::string>::setUp(references, 15);
  }

  std::vector<NetworkFixture> network;
};

TEST_F(NetworkTest, push) {
  for (std::size_t sender = 0; sender < network.size(); ++sender) {
    network[sender].push("SomeMessage");

    for (std::size_t receiver = 0; receiver < network.size(); ++receiver) {
      if (receiver == sender) {
        continue;
      }
      std::vector<std::string> polled;
      for (auto &in : network[receiver].incoming) {
        auto delta = consume(*in, &always_true);
        polled.insert(polled.end(), delta.begin(), delta.end());
      }
      ASSERT_EQ(polled.size(), 1);
      ASSERT_EQ(polled.front(), "SomeMessage");
    }
  }
}

TEST_F(NetworkTest, poll_lessThanPushed) {
  network.front().push("FirstMessage");
  network.front().push("SecondMessage");
  network.front().push("ThirdMessage");
  network.front().push("FourthMessage");

  std::for_each(
      network.begin() + 1, network.end(), [](Network<std::string> &el) {
        std::vector<std::string> polled;
        el.poll(
            2, [&polled](const std::string &msg) { polled.emplace_back(msg); });
        std::vector<std::string> expected{"FirstMessage", "SecondMessage"};
        ASSERT_EQ(polled, expected);
      });
}

TEST_F(NetworkTest, poll_moreThanPushed) {
  network.front().push("FirstMessage");
  network.front().push("SecondMessage");

  std::for_each(
      network.begin() + 1, network.end(), [](Network<std::string> &el) {
        std::vector<std::string> polled;
        el.poll(
            4, [&polled](const std::string &msg) { polled.emplace_back(msg); });
        std::vector<std::string> expected{"FirstMessage", "SecondMessage"};
        ASSERT_EQ(polled, expected);
      });
}

// TODO test that msg remain in dequeue, then consumed then polled then received
// actaully
