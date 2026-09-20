/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <cstring>
#include <span>

namespace mt_rrt {
template <typename T> class ObjectPool {
public:
  ObjectPool(std::size_t chunk_capacity = 1000) chunk_capacity_{chunk_capacity},
      head_{Chunk::make(chunk_capacity_)}, tail_{head_} {}

  ~ObjectPool() {
    for (Chunk *current = root; current; current = current->next) {
      delete[] current->buffer;
    }
  }

  ObjectPool(const ObjectPool &) = delete;
  ObjectPool &operator=(const ObjectPool &) = delete;

  ObjectPool(ObjectPool &&) = delete;
  ObjectPool &operator=(ObjectPool &&) = delete;

  void push(std::span<const T> to_add) {
    std::size_t residual = chunk_capacity_ - tail_->len;
    if (residual < to_add.size()) {
      auto *next = Chunk::make(chunk_capacity_);
      tail_->next = next;
      tail_ = next;
    }
    std::memcpy(tail_->buffer + tail_->len, to_add.data(),
                sizeof(T) * to_add.size());
  }

private:
  struct Chunk {
    Chunk *make(std::size_t cap) {
      return new Chunk{.buffer = new T[cap], .len = 0};
    }

    Chunk *next{nullptr};
    T *buffer{nullptr};
    std::size_t len;
  };

  std::size_t chunk_capacity_;
  Chunk *head_;
  Chunk *tail_;
};
} // namespace mt_rrt
