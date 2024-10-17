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
template <typename T> struct ChunkBase {
  ChunkBase(std::size_t capacity) : capacity_{capacity} {
    buffer = new char[sizeof(T) * capacity];
  }
  ~ChunkBase() { delete[] buffer; }

  T *at(std::size_t pos) {
    T *ptr = reinterpret_cast<T *>(buffer);
    return &ptr[pos];
  }

  std::size_t capacity_;
  std::size_t size_ = 0;
  char *buffer = nullptr;
};

template <typename T> struct Chunk : public ChunkBase<T> {
  using ChunkBase<T>::ChunkBase;

  ~Chunk() {
    T *ptr = reinterpret_cast<T *>(buffer);
    for (std::size_t k = 0; k < size_; ++k, ++ptr) {
      ptr->~T();
    }
  }
};

struct Chunk<int> : ChunkBase<int> {};
struct Chunk<std::int16_t> : ChunkBase<std::int16_t> {};
struct Chunk<std::int32_t> : ChunkBase<std::int32_t> {};
struct Chunk<std::int64_t> : ChunkBase<std::int64_t> {};
struct Chunk<std::uint16_t> : ChunkBase<std::uint16_t> {};
struct Chunk<std::uint32_t> : ChunkBase<std::uint32_t> {};
struct Chunk<std::uint64_t> : ChunkBase<std::uint64_t> {};
struct Chunk<float> : ChunkBase<float> {};
struct Chunk<double> : ChunkBase<double> {};

template <typename T> class ObjectPool {
public:
  ObjectPool();

  template <typename... Args>
  T &emplace_back(Args &&...args); // TODO this for normal types

  T *emplace_back_multiple(
      const T *source,
      std::size_t how_many); // TODO this only for trivial types

  static const inline std::size_t INITIAL_CAPACITY = 100;

private:
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
