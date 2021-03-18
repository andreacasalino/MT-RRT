/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_CART_H
#define MT_RRT_SAMPLE_NAVIGATION_CART_H

#include <Sphere.h>
#include <Limited.h>
#include <Checker.h>
#include <array>
#include <memory>

namespace mt::sample {
    class Cart {
    public:
        Cart(const Positive<float>& width, const Positive<float>& length);

        // pose buffer should be [x,y,angle]
        bool isColliding(const float* pose, const geometry::Sphere& obstacle) const;

        const float getWidth() const;
        const float getLength() const;

    private:
        typedef std::unique_ptr<geometry::Segment> SegmentPtr;

    // data
        std::array<geometry::Point, 4> vertices;
        std::array<SegmentPtr, 4> segments;

        mutable geometry::Point relativePos;
        mutable geometry::SegmentPointChecker checker0;
        mutable geometry::SegmentPointChecker checker1;
    };
}

#endif