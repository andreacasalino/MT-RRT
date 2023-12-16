/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <cstring>
#include <memory>
#include <type_traits>
#include <vector>

namespace mt_rrt {
template <typename T> class ObjectPool {
public:
  ObjectPool();

  template <typename... Args> T &emplace_back(Args &&...args);

  T *emplace_back_multiple(const T *source, std::size_t how_many);

  static const inline std::size_t INITIAL_CAPACITY = 100;

private:
  struct Chunk {
    Chunk(std::size_t capacity) : capacity_{capacity} {
      buffer = new char[sizeof(T) * capacity];
    }
    ~Chunk() {
      if constexpr (!(std::is_same_v<T, int> || std::is_same_v<T, float> ||
                      std::is_same_v<T, double>)) // in principle others like
                                                  // std::uintXX
      {
        T *ptr = reinterpret_cast<T *>(buffer);
        for (std::size_t k = 0; k < size_; ++k, ++ptr) {
          ptr->~T();
        }
      }
      delete[] buffer;
    }

    T *at(std::size_t pos) {
      T *ptr = reinterpret_cast<T *>(buffer);
      return &ptr[pos];
    }

    std::size_t capacity_;
    std::size_t size_ = 0;
    char *buffer = nullptr;
  };

  std::vector<std::unique_ptr<Chunk>> chunks_;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T> ObjectPool<T>::ObjectPool() {
  chunks_.emplace_back(std::make_unique<Chunk>(INITIAL_CAPACITY));
}

template <typename T>
template <typename... Args>
T &ObjectPool<T>::emplace_back(Args &&...args) {
  Chunk *recipient = chunks_.back().get();
  if (recipient->size_ == recipient->capacity_) {
    // create a new chunk
    recipient =
        chunks_.emplace_back(std::make_unique<Chunk>(recipient->capacity_ * 10))
            .get();
  }
  T *slot = recipient->at(recipient->size_++);
  new (slot) T{std::forward<Args>(args)...};
  return *slot;
}

template <typename T>
T *ObjectPool<T>::emplace_back_multiple(const T *source, std::size_t how_many) {
  Chunk *recipient = chunks_.back().get();
  if (std::size_t residual_size = recipient->capacity_ - recipient->size_;
      residual_size < how_many) {
    std::size_t next_chunk_cap =
        std::max<std::size_t>(how_many, recipient->capacity_ * 10);
    // create a new chunk
    recipient =
        chunks_.emplace_back(std::make_unique<Chunk>(next_chunk_cap)).get();
  }
  T *slot = recipient->at(recipient->size_);
  std::memcpy(slot, source, how_many * sizeof(T));
  recipient->size_ += how_many;
  return slot;
}
} // namespace mt_rrt
