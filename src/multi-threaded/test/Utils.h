/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <atomic>
#include <functional>
#include <thread>
#include <vector>

namespace mt_rrt {
class Person {
public:
  Person(const std::string &name, const std::string &surname)
      : name{name}, surname{surname} {}

  Person(const Person &) = delete;
  Person &operator=(const Person &) = delete;

  Person(Person &&) = default;
  Person &operator=(Person &&) = default;

  bool operator==(const Person &o) const {
    return (name == o.name) && (surname == o.surname);
  }

  const auto &getName() const { return name; }
  const auto &getSurName() const { return surname; }

private:
  std::string name;
  std::string surname;
};

class ParallelRegion {
public:
  template <typename Task_front, typename Task_second, typename... Tasks>
  static void execute(Task_front &&front, Task_second &&second,
                      Tasks &&...others) {
    ParallelRegion region;
    region.add(std::forward<Task_front>(front));
    region.add(std::forward<Task_second>(second));
    (region.add(std::forward<Tasks>(others)), ...);
    region.run();
  }

private:
  ParallelRegion() = default;

  template <typename Pred> void add(Pred &&to_add) {
    tasks.emplace_back(std::forward<Pred>(to_add));
  }

  void run() {
    std::atomic<std::size_t> started = 0;
    auto wait = [&started, tot = tasks.size()]() {
      ++started;
      while (started.load() != tot) {
      }
    };

    std::vector<std::thread> workers;
    std::for_each(tasks.begin() + 1, tasks.end(), [&](const auto &pred) {
      workers.emplace_back([&]() {
        wait();
        pred();
      });
    });
    wait();
    tasks.front()();

    for (auto &w : workers)
      w.join();
  }

  std::vector<std::function<void()>> tasks;
};
} // namespace mt_rrt
