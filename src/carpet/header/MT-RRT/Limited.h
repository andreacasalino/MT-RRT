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
/**
 * @brief A real number whose value should always be positive
 */
template <typename T, T LowerBound, T UpperBound> class Limited {
public:
  Limited(T val) { set(val); };

  /** @return the current value
   */
  inline T get() const { return this->value; };

  /** @param the new value to assumed
   *  @throw if the value is not consistent with the bounds
   */
  void set(const T &newValue) {
    if constexpr (std::numeric_limits<T>::min() < LowerBound) {
      if (newValue < LowerBound) {
        throw Error::make("value is lower than minum which is {}", LowerBound);
      }
    }

    if constexpr (UpperBound < std::numeric_limits<T>::max()) {
      if (UpperBound < newValue) {
        throw Error::make("value is higher than maximum which is {}",
                          UpperBound);
      }
    }

    this->value = newValue;
  };

protected:
  T value;
};

using Positive = Limited<float, 0.f, std::numeric_limits<float>::max()>;
} // namespace mt_rrt
