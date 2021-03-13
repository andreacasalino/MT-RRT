/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_CHECKER_H
#define MT_RRT_SAMPLE_CHECKER_H

#include <Point.h>

namespace mt::sample::geometry {
    class Checker {
    public:
        inline const float& getDistance() { return this->distance; };
        inline const Point& getClosesetInA() { return this->closestInA; };
        inline const Point& getClosesetInB() { return this->closestInB; };

    protected:
        float	distance;
        Point	closestInA;
        Point	closestInB;
    };

    class LinePointChecker : public Checker {
    public:
        LinePointChecker() = default;

        virtual void check(const Segment& line, const Point& point);

        //closest point to sphere is obtained as the following convex combination:
        //closest = seg.V1 + s * (seg.V2 - seg.V1)
        inline const float& getCoeff() { return this->coeff; };

    protected:
        void computeCoeff(const Segment& line, const Point& point);
        void computeInfo(const Segment& line, const Point& point);
        
        float		coeff;
    };

    class SegmentPointChecker : public LinePointChecker {
    public:
        SegmentPointChecker() = default;

        void check(const Segment& seg, const Point& point) override;
    };

    class LineLineChecker : public Checker {
    public:
        LineLineChecker() = default;

        virtual void check(const Segment& lineA, const Segment& lineB);

        inline bool wereParallel() { return this->parallelism; };

        //closest point in segA is obtained as the following convex combination:
        //closest = segA.V1 + a * (segA.V2 - segA.V1)
        inline const float& getCoeffA() { return this->coeffA; };
        //closest point in segB is obtained as the following convex combination:
        //closest = segB.V1 + b * (segB.V2 - segB.V1)
        inline const float& getCoeffB() { return this->coeffB; };

    protected:
        void computeCoeff(const Segment& lineA, const Segment& lineB);
        void computeInfo(const Segment& lineA, const Segment& lineB);

        bool		parallelism = false;
        float		coeffA;
        float		coeffB;
    };

    class SegmentSegmentChecker : public LineLineChecker {
    public:
        SegmentSegmentChecker() = default;

        void check(const Segment& lineA, const Segment& lineB) override;

    private:
        void checkVertices(const Segment& segA, const Segment& segB);
    };
}

#endif