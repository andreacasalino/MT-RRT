/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Sphere.h>
#include <Error.h>

namespace mt::sample::geometry {
    Sphere::Sphere(const float& x, const float& y, const float& ray)
        : center(x, y) {
        this->ray.set(ray);
    }
}