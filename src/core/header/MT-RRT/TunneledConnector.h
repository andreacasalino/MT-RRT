/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>

namespace mt_rrt {
float euclidean_distance(const View &a, const View &b);

/**
 * @brief Implements the approach described at
 * Section "Tunneled check collision" of the documentation.
 * Optimal trajectories \tau are assumed to be simple line in the space of
 * configuration. Every advancement move the cursor of a qunatum of space along
 * the trajectory and only the reached state is checked to be in the admitted
 * space (and not the segment going from the previous state to the reached one).
 */
class TunneledConnector : public Connector {
public:
  TunneledConnector(const std::size_t size);

  float minCost2Go(const View &start, const View &end) const override;

  TrajectoryPtr getTrajectory(const View &start,
                              const View &end) const override;

  std::size_t getStateSpaceSize() const { return space_size; }

  float getSteerDegree() const { return steer_degree.get(); }
  void setSteerDegree(const Positive<float> &degree) {
    steer_degree.set(degree.get());
  }

  /**
   * @brief Returns true is the advanced_state is not admitted
   */
  virtual bool checkAdvancement(const View &previous_state,
                                const View &advanced_state) const = 0;

protected:
  TunneledConnector(const TunneledConnector &o);

  class Line;

private:
  const std::size_t space_size;
  Positive<float> steer_degree = 0.1f;
};

class TunneledConnector::Line : public Trajectory {
public:
  Line(const View &start, const View &end, const TunneledConnector &caller);

  AdvanceInfo advance() final;

  View getState() const final { return View{actual}; };

  float getCumulatedCost() const final;

private:
  const TunneledConnector &caller;

  View start;
  std::vector<float> actual;
  View target;

  std::vector<float> advanced;
};

} // namespace mt_rrt
