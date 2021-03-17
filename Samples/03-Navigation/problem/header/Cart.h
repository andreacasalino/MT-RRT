/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_H

#include <Sphere.h>

namespace mt::sample {
    class Cart {
    public:
        Cart(const float& width, const float& length);

        // pose buffer should be [x,y,angle]
        bool isColliding(const float* pose, const geometry::Sphere& obstacle) const;

        inline const float& getWidth() const { return this->width; };
        inline const float& getLength() const { return this->length; };

    private:
        float width;
        float length;
    };
}

#endif