/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <atomic>
#include <type_traits>

namespace mt_rrt {
template <typename T> class LockFreeForwardList {
public:
  template <typename... Args> LockFreeForwardList(Args &&...args_front);

  ~LockFreeForwardList();

  template <typename... Args> void emplace_back(Args &&...args);

  template <typename Pred> void forEach(Pred &&pred);

  std::size_t size() const { return size_.load(); }

private:
  struct Node_ {
    template <typename... Args>
    Node_(Args &&...args) : value{std::forward<Args>(args)...} {}

    T value;
    std::atomic<Node_ *> next = nullptr;
  };

  Node_ *front;
  std::atomic<Node_ *> tail;
  std::atomic<std::size_t> size_;
};

///////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
template <typename... Args>
LockFreeForwardList<T>::LockFreeForwardList(Args &&...args_front)
    : front{new Node_{std::forward<Args>(args_front)...}}, tail{front},
      size_{1} {}

template <typename T>
template <typename... Args>
void LockFreeForwardList<T>::emplace_back(Args &&...args) {
  Node_ *to_add = new Node_{std::forward<Args>(args)...};
  Node_ *cursor = tail.load(std::memory_order::memory_order_acquire);
  while (true) {
    Node_ *expected = nullptr;
    if (cursor->next.compare_exchange_strong(expected, to_add, std::memory_order::memory_order_acquire)) {
      tail.store(to_add, std::memory_order::memory_order_acquire);
      break;
    } else {
      cursor = expected;
    }
  }
  ++size_;
}

template <typename T>
template <typename Pred>
void LockFreeForwardList<T>::forEach(Pred &&pred) {
  for (Node_ *cursor = front; cursor != nullptr; cursor = cursor->next.load(std::memory_order::memory_order_acquire)) {
    if constexpr (std::is_pointer_v<T>) {
      pred(cursor->value);
    }
    else {
      pred(std::ref(cursor->value));
    }
  }
}

template <typename T> LockFreeForwardList<T>::~LockFreeForwardList() {
  Node_ *cursor = front, *next;
  while (cursor != nullptr) {
    next = cursor->next.load();
    delete cursor;
    cursor = next;
  }
}
} // namespace mt_rrt
