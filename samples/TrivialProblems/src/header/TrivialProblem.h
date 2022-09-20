/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>
#include <MT-RRT-core/TunneledConnector.h>

#include <array>
#include <optional>

namespace mt_rrt::samples {
class Transform {
public:
  using Traslation = std::array<float, 2>;

  Transform(const std::optional<float> &rotation_angle,
            const std::optional<Traslation> &traslation);

  float getAngle() const {
    return atan2f(rotation.sin_angle, rotation.cos_angle);
  }
  const Traslation &getTraslation() const { return traslation; };

  static Transform combine(const Transform &pre, const Transform &post);

  static Transform rotationAroundCenter(float rotation_angle,
                                        const Traslation &center);

  State seenFromRelativeFrame(const State &subject) const;

  void dotRotationMatrix(float *recipient, const float *point) const;
  void dotRotationMatrixTrasp(float *recipient, const float *point) const;

private:
  struct RotationInfo {
    float cos_angle;
    float sin_angle;
  };
  RotationInfo rotation;
  Traslation traslation;
};

struct Box {
  Box(const State &min, const State &max,
      const std::optional<Transform> &trsf = std::nullopt);

  State min_corner; // seen from local frame!!!
  State max_corner; // seen from local frame!!!

  std::optional<Transform> trsf;
};
using Boxes = std::vector<Box>;

bool collides(const State &segment_start, const State &segment_end,
              const Box &box);

// Universe is a (-1 , -1) x (1 , 1) box, with steer radius
// equal to 0.05
class TrivialProblemConnector : public TunneledConnector {
public:
  TrivialProblemConnector(const Boxes &obstacles);

  TrivialProblemConnector(const TrivialProblemConnector &o);

  const Boxes &getBoxes() const { return *obstacles; }

  std::unique_ptr<Connector> copy() const final {
    return std::make_unique<TrivialProblemConnector>(*this);
  }

  // equal to 0.05
  static const float STEER_DEGREE;

protected:
  bool checkAdvancement(const State &previous_state,
                        const State &advanced_state) const override;

  using BoxesPtr = std::shared_ptr<const Boxes>;
  BoxesPtr obstacles;
};

std::shared_ptr<ProblemDescription>
make_trivial_problem_description(const std::optional<Seed> &seed,
                                 const Boxes &obstacles);
} // namespace mt_rrt::samples
