/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_SPHERE_H
#define MT_RRT_SAMPLE_SPHERE_H

#include <Point.h>
#include <Limited.h>

namespace mt::sample::geometry {
    class Sphere {
    public:
        Sphere(const float& x, const float& y, const float& ray);

        Sphere(const Sphere&) = default;
        
        inline const geometry::Point& getCenter() const { return this->center; };
        inline float getRay() const { return this->ray.get(); };

    private:
        geometry::Point center;
        mt::Positive<float> ray;
    };
}

#endif
