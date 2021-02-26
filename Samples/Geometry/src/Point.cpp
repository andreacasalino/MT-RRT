/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/Point.h"

namespace mt::sample {
    Point::Point() {
        this->coordinates[0] = // todo rand value
        this->coordinates[1] = // todo rand value
        this->coordinates[2] = // todo rand value
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
}