/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/Point.h"
#include <random>

namespace mt::sample::geometry {
    Point::Point() {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.f, 1.f);
        this->coordinates[0] = distribution(generator);
        this->coordinates[1] = distribution(generator);
        this->coordinates[2] = distribution(generator);
    };

    Point::Point(const float& x, const float& y) {
        this->coordinates[0] = x;
        this->coordinates[1] = y;
        this->coordinates[2] = 0.f;
    };

    Point::Point(const float& x, const float& y, const float& z)
        : Point(x,y) {
        this->coordinates[2] = z;
    };

    Point::Point(const Point& o) {
        *this = o;
    };

    Point& Point::operator=(const Point& o) {
        for (std::size_t k = 0; k < 3; ++k) {
            this->coordinates[k] = o.coordinates[k];
        }
        return *this;
    };
}