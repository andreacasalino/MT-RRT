/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Error.h>
#include <limits>

namespace mt_rrt {
namespace detail {
template <typename T> class ValueAware {
public:
  T get() const { return val_; }

protected:
  T val_;
};

template <typename T> class UpperBoundAware {
protected:
  UpperBoundAware(T bound) : bound_{bound} {}

  void check(T val) const {
    if (bound_ < val) {
      throw Error("inconsistent value: too big");
    }
  }

private:
  T bound_;
};

template <typename T> class LowerBoundAware {
protected:
  LowerBoundAware(T bound) : bound_{bound} {}

  void check(T val) const {
    if (val < bound_) {
      throw Error("inconsistent value: too low");
    }
  }

private:
  T bound_;
};
} // namespace detail

/**
 * @brief A quantity whose value should always be below an upper threshold
 */
template <typename T>
class UpperLimited : public detail::UpperBoundAware<T>,
                     public detail::ValueAware<T> {
public:
  UpperLimited(T bound, T val) : detail::UpperBoundAware<T>{bound} {
    this->set(val);
  };

  UpperLimited(T val) : UpperLimited{val, val} {}

  void set(T newValue) {
    this->chek(newValue);
    this->val_ = newValue;
  };
};

/**
 * @brief A quantity whose value should always be above a lower threshold
 */
template <typename T>
class LowerLimited : public detail::LowerBoundAware<T>,
                     public detail::ValueAware<T> {
public:
  LowerLimited(T bound, T val) : detail::LowerBoundAware<T>{bound} {
    this->set(val);
  };

  LowerLimited(T val) : LowerLimited{val, val} {}

  void set(T newValue) {
    this->check(newValue);
    this->val_ = newValue;
  };
};

/**
 * @brief A quantity whose value should always remain between defined
 * bounds
 */
template <typename T>
class Limited : public detail::UpperBoundAware<T>,
                public detail::LowerBoundAware<T>,
                public detail::ValueAware<T> {
public:
  Limited(T l, T u, T val)
      : detail::UpperBoundAware<T>{u}, detail::LowerBoundAware<T>{l} {
    this->set(val);
  };

  void set(T newValue) {
    this->detail::UpperBoundAware<T>::check(newValue);
    this->detail::LowerBoundAware<T>::check(newValue);
    this->val_ = newValue;
  };
};

template <typename T> class Positive : public detail::ValueAware<T> {
public:
  Positive(T val = T{0}) { this->set(val); }

  void set(T newValue) {
    if (newValue < 0) {
      throw Error("negative value not allowed");
    }
    this->val_ = newValue;
  }
};
} // namespace mt_rrt
