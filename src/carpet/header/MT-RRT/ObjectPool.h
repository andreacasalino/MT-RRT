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
    T *ptr = reinterpret_cast<T *>(this->buffer);
    for (std::size_t k = 0; k < this->size_; ++k, ++ptr) {
      ptr->~T();
    }
  }
};

template<>
struct Chunk<int> : ChunkBase<int> {};
template<>
struct Chunk<float> : ChunkBase<float> {};
template<>
struct Chunk<double> : ChunkBase<double> {};
template<>
struct Chunk<std::int16_t> : ChunkBase<std::int16_t> {};
template<>
struct Chunk<std::int64_t> : ChunkBase<std::int64_t> {};
template<>
struct Chunk<std::uint16_t> : ChunkBase<std::uint16_t> {};
template<>
struct Chunk<std::uint32_t> : ChunkBase<std::uint32_t> {};
template<>
struct Chunk<std::uint64_t> : ChunkBase<std::uint64_t> {};

template <typename T> class ObjectPoolBase {
public:
  ObjectPoolBase() {
    chunks_.emplace_back(std::make_unique<Chunk<T>>(INITIAL_CAPACITY));
  }

  static const inline std::size_t INITIAL_CAPACITY = 100;

protected:
  std::vector<std::unique_ptr<Chunk<T>>> chunks_;
};

template <typename T>
class ObjectPool : public ObjectPoolBase<T> {
public:
  using ObjectPoolBase<T>::ObjectPoolBase;

  template <typename... Args>
  T &emplace_back(Args &&...args);
};

template <typename T>
class ObjectPoolMemCopiableTypes : public ObjectPoolBase<T> {
public:
  using ObjectPoolBase<T>::ObjectPoolBase;

  T *emplace_back(const T *source_buffer, std::size_t buffer_size);
};

template<>
struct ObjectPool<int> : ObjectPoolMemCopiableTypes<int> {};
template<>
struct ObjectPool<float> : ObjectPoolMemCopiableTypes<float> {};
template<>
struct ObjectPool<double> : ObjectPoolMemCopiableTypes<double> {};
template<>
struct ObjectPool<std::int16_t> : ObjectPoolMemCopiableTypes<std::int16_t> {};
template<>
struct ObjectPool<std::int64_t> : ObjectPoolMemCopiableTypes<std::int64_t> {};
template<>
struct ObjectPool<std::uint16_t> : ObjectPoolMemCopiableTypes<std::uint16_t> {};
template<>
struct ObjectPool<std::uint32_t> : ObjectPoolMemCopiableTypes<std::uint32_t> {};
template<>
struct ObjectPool<std::uint64_t> : ObjectPoolMemCopiableTypes<std::uint64_t> {};

/////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
template <typename... Args>
T &ObjectPool<T>::emplace_back(Args &&...args) {
  Chunk<T> *recipient = this->chunks_.back().get();
  if (recipient->size_ == recipient->capacity_) {
    // create a new chunk
    recipient =
        this->chunks_.emplace_back(std::make_unique<Chunk<T>>(recipient->capacity_ * 10))
            .get();
  }
  T *slot = recipient->at(recipient->size_++);
  new (slot) T{std::forward<Args>(args)...};
  return *slot;
}

template <typename T>
T *ObjectPoolMemCopiableTypes<T>::emplace_back(const T *source_buffer, std::size_t buffer_size) {
  Chunk<T> *recipient = this->chunks_.back().get();
  if (std::size_t residual_size = recipient->capacity_ - recipient->size_;
      residual_size < buffer_size) {
    std::size_t next_chunk_cap =
        std::max<std::size_t>(buffer_size, recipient->capacity_ * 10);
    // create a new chunk
    recipient =
        this->chunks_.emplace_back(std::make_unique<Chunk<T>>(next_chunk_cap)).get();
  }
  T *slot = recipient->at(recipient->size_);
  std::memcpy(slot, source_buffer, buffer_size * sizeof(T));
  recipient->size_ += buffer_size;
  return slot;
}
} // namespace mt_rrt
