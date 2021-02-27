/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_POINT_H
#define MT_RRT_SAMPLE_POINT_H

namespace mt::sample::geometry {
    class Point {
    public:
        Point(); // random point
        Point(const float& x, const float& y);
        Point(const float& x, const float& y, const float& z);

        Point(const Point& ) = default;
        Point& operator=(const Point& ) = default;

        inline float& x() { return this->coordinates[0]; }
        inline const float& x() const { return this->coordinates[0]; }
        inline float& y() { return this->coordinates[1]; }
        inline const float& y() const { return this->coordinates[1]; }
        inline float& z() { return this->coordinates[2]; }
        inline const float& z() const { return this->coordinates[2]; }

        inline float* data() { return this->coordinates; };
        inline const float* data() const { return this->coordinates; };

    private:
        float coordinates[3];
    };

    struct Segment {
        Point  A;
        Point  B;
    };
}

#endif