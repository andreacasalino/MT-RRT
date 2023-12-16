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
 * @brief A quantity whose value should always remain between defined
 * bounds
 */
template <typename T> class Limited {
public:
  /** @param lower bound for the value
   *  @param upper bound for the value
   *  @param initial value to set
   */
  Limited(const T &lowerBound, const T &upperBound, const T &initialValue)
      : lowerBound(lowerBound), upperBound(upperBound) {
    if (this->lowerBound >= this->upperBound) {
      throw Error("inconsistent bounds");
    }
    this->set(initialValue);
  };

  /** @brief similar to Limited::Limited(const T& lowerBound, const T&
   * upperBound, const T& initialValue), assuming lowerBound as initial value
   */
  template <bool LowerOrUpper = true>
  Limited(const T &lowerBound, const T &upperBound)
      : Limited(lowerBound, upperBound,
                LowerOrUpper ? lowerBound : upperBound){};

  Limited(const Limited &) = default;
  Limited &operator=(const Limited &) = default;

  inline T getLowerBound() const { return this->lowerBound; };
  inline T getUpperBound() const { return this->upperBound; };

  /** @return the current value
   */
  inline T get() const { return this->value; };

  /** @param the new value to assumed
   *  @throw if the value is not consistent with the bounds
   */
  void set(const T &newValue) {
    if (newValue < this->lowerBound) {
      throw Error{"value is lower than minum which is ", lowerBound};
    }
    if (newValue > this->upperBound) {
      throw Error{"value is higher than maximum which is ", upperBound};
    }
    this->value = newValue;
  };

protected:
  T value;
  T lowerBound;
  T upperBound;
};

/**
 * @brief A @Limited quantity, having infinity as upper bound
 */
template <typename T> class LowerLimited : public Limited<T> {
public:
  LowerLimited(const T &lowerBound, const T &initialValue)
      : Limited<T>(lowerBound, std::numeric_limits<T>::max(), initialValue){};

  LowerLimited(const T &lowerBound) : LowerLimited(lowerBound, lowerBound){};
};

/**
 * @brief A @LowerLimited quantity, having 0.0 as lower bound
 */
template <typename T> class Positive : public LowerLimited<T> {
public:
  Positive(const T &initialValue = static_cast<T>(0))
      : LowerLimited<T>(static_cast<T>(0), initialValue){};
};

/**
 * @brief A @Limited quantity, having negative infinity as lower bound
 */
template <typename T> class UpperLimited : public Limited<T> {
public:
  UpperLimited(const T &upperBound, const T &initialValue)
      : Limited<T>(std::numeric_limits<T>::min(), upperBound, initialValue){};

  UpperLimited(const T &upperBound) : UpperLimited(upperBound, upperBound){};
};
} // namespace mt_rrt
