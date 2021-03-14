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
#include <Limited.h>

namespace mt::sample {
    typedef std::shared_ptr<const geometry::Point> PointShared;

    class Capsule {
    public:
        Capsule(const float& ray, PointShared pointA, PointShared pointB)
            : ray(ray)
            , pointA(pointA)
            , pointB(pointB) {
        };

        float ray;
        PointShared pointA;
        PointShared pointB;
    };

    class Manipulator {
    public:
        struct Link {
            mt::Positive<float> length;
            mt::Positive<float> ray;
        };
        Manipulator(const geometry::Point& base, const std::vector<Link>& lenghts);

        inline std::size_t dof() const { return this->links.size(); };

        static std::size_t dofTot(const std::vector<Manipulator>& robots);

        std::vector<Capsule> directKinematics(const float* pose) const;

        inline const geometry::Point& getBase() const { return this->base; };

        inline const std::vector<Link>& getLinks() const { return this->links; };

    private:
        geometry::Point base;
        std::vector<Link> links;
    };

    Manipulator make_manipulator(const std::vector<float>& data);
}

#endif