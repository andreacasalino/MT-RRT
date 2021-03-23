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
    /** @brief Object used to check collision of pair of geometries
	 */
    class Checker {
    public:
        inline const float& getDistance() { return this->distance; };
        inline const Point& getClosesetInA() { return this->closestInA; };
        inline const Point& getClosesetInB() { return this->closestInB; };

    protected:
        /** @brief The minimum distance of the last checked pair. 0 when a collision is present
	     */
        float	distance;
        /** @brief Closest points of the last checked pair. Meaningful if the last pair did not collide
	     */
        Point	closestInA;
        Point	closestInB;
    };

    /** @brief Infinite line against a sphere
	 */
    class LinePointChecker : public Checker {
    public:
        LinePointChecker() = default;

        virtual void check(const Segment& line, const Point& point);

        /** @brief closest point in the line is obtained as the following convex combination:
         * closest = line.A + a * (line.A - line.B)
	     */
        inline const float& getCoeff() { return this->coeff; };

    protected:
        void computeCoeff(const Segment& line, const Point& point);
        void computeInfo(const Segment& line, const Point& point);
        
        float		coeff;
    };

    /** @brief Finite segment against a sphere
	 */
    class SegmentPointChecker : public LinePointChecker {
    public:
        SegmentPointChecker() = default;

        void check(const Segment& seg, const Point& point) override;
    };

    /** @brief Infinite line against another infinite line
	 */
    class LineLineChecker : public Checker {
    public:
        LineLineChecker() = default;

        virtual void check(const Segment& lineA, const Segment& lineB);

        inline bool wereParallel() { return this->parallelism; };

        /** @brief closest point in segA is obtained as the following convex combination:
         * closest = segA.A + a * (segA.A - segA.B)
	     */
        inline const float& getCoeffA() { return this->coeffA; };
        /** @brief closest point in segB is obtained as the following convex combination:
         * closest = segB.A + a * (segB.A - segBS.B)
	     */
        inline const float& getCoeffB() { return this->coeffB; };

    protected:
        void computeCoeff(const Segment& lineA, const Segment& lineB);
        void computeInfo(const Segment& lineA, const Segment& lineB);

        bool		parallelism = false;
        float		coeffA;
        float		coeffB;
    };

    /** @brief Finite segment against another finite segment
	 */
    class SegmentSegmentChecker : public LineLineChecker {
    public:
        SegmentSegmentChecker() = default;

        void check(const Segment& lineA, const Segment& lineB) override;

    private:
        void checkVertices(const Segment& segA, const Segment& segB);
    };
}

#endif