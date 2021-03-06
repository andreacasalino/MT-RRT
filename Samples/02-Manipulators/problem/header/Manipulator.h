/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_H
#define MT_RRT_SAMPLE_MANIPULATOR_H

#include <Point.h>
#include <vector>
#include <memory>

namespace mt::sample {
    typedef std::shared_ptr<const geometry::Point> PointShared;

    class Capsule {
    public:
        Capsule(const float& ray, PointShared pointA, PointShared pointB)
            : ray(ray)
            , pointA(pointA)
            , pointB(pointB) {
        };

        const float& ray;
        const PointShared pointA;
        const PointShared pointB;
    };

    class Manipulator {
    public:
        struct Link {
            float length;
            float ray;
        };
        Manipulator(const geometry::Point& base, const std::vector<Link>& lenghts);

        inline std::size_t dof() const { return this->links.size(); };

        static std::size_t dofTot(const std::vector<Manipulator>& robots);

        std::vector<Capsule> directKinematics(const float* pose) const;

    private:
        geometry::Point base;
        std::vector<Link> links;
    };
}

#endif