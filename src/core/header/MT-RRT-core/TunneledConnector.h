/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>

namespace mt_rrt {
float euclidean_distance(const State &a, const State &b);

class TunneledConnector : public Connector {
public:
  TunneledConnector(const std::size_t size);

  float minCost2Go(const State &start, const State &end) const override;

  TrajectoryPtr getTrajectory(const State &start,
                              const State &end) const override;

  std::size_t getStateSpaceSize() const { return space_size; }

  float getSteerDegree() const { return steer_degree.get(); }
  void setSteerDegree(const Positive<float> &degree) {
    steer_degree.set(degree.get());
  }

  virtual bool checkAdvancement(const State& previous_state,
      const State& advanced_state) const = 0;

protected:
  TunneledConnector(const TunneledConnector &o);

  class Line;

private:
  const std::size_t space_size;
  Positive<float> steer_degree = 0.1f;
};

class TunneledConnector::Line : public Trajectory {
public:
  Line(const State &start, const State &end, const TunneledConnector &caller);

  AdvanceInfo advance() final;

  State getState() const final { return attual; };

  float getCumulatedCost() const final;

private:
  const TunneledConnector &caller;

  const State start;
  State attual;
  const State target;
};
} // namespace mt_rrt
