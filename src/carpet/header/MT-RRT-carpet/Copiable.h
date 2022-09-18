/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <memory>

namespace mt_rrt {
/** @brief Interface for a copiable object.
 */
template <typename T> class Copiable {
public:
  virtual ~Copiable() = default;

  /** @brief A deep copy need to be implemented for the descendant
   */
  virtual std::unique_ptr<T> copy() const = 0;

protected:
  Copiable() = default;
};
} // namespace mt_rrt
