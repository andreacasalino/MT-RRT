/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Manipulator.h>
#include <Error.h>
#include <list>
#include <math.h>

namespace mt::sample {
    const std::vector<Manipulator::Link>& checkLenghts(const std::vector<Manipulator::Link>& links) {
        for (auto it = links.begin(); it != links.end(); ++it) {
            if (it->length < 0.f) {
                throw Error("negative lenght is not possible");
            }
            if (it->ray < 0.f) {
                throw Error("negative ray is not possible");
            }
        }
        return links;
    }

    Manipulator::Manipulator(const geometry::Point& base, const std::vector<Link>& links)
        : base(base)
        , links(checkLenghts(links)) {
        if (this->links.empty()) {
            throw Error("robot shoudl have at least one link");
        }
    }

    std::vector<Capsule> Manipulator::directKinematics(const float* pose) const {
        std::vector<PointShared> points;
        points.reserve(this->links.size() + 1);
        points.emplace_back(std::make_shared<geometry::Point>(this->base));
        float cumulAngle = 0.f;
        for (std::size_t k = 0; k < this->links.size(); ++k) {
            cumulAngle += pose[k];
            points.emplace_back(std::make_shared<geometry::Point>(points.back()->x() + cosf(cumulAngle) * this->links[k].length, 
                                                                  points.back()->y() + sinf(cumulAngle) * this->links[k].length));
        }

        std::vector<Capsule> capsules;
        capsules.reserve(this->links.size());
        for (std::size_t k = 0; k < this->links.size(); ++k) {
            capsules.emplace_back(this->links[k].ray, points[k], points[k + 1]);
        }
        return capsules;
    }

    std::size_t Manipulator::dofTot(const std::vector<Manipulator>& robots) {
        std::size_t dof = 0;
        for (auto it = robots.begin(); it != robots.end(); ++it) {
            dof += it->dof();
        }
        return dof;
    }

    Manipulator make_manipulator(const std::vector<float>& data) {
        if(data.size() < 4) throw Error("invalid manipulator data");
        if(data.size() % 2 != 0) throw Error("invalid manipulator data");
        std::size_t dof =  (data.size() - 2)/ 2;
        geometry::Point base(data[0] , data[1]);
        std::vector<Manipulator::Link> links;
        links.resize(dof);
        for(std::size_t k=2; k<data.size(); k += 2) {
            links[k].length = data[k];
            links[k].ray = data[k+1];
        }
        return Manipulator(base, links);
    }

    NodeState degree2rad(const NodeState& pose) {
        NodeState converted = pose;
        for(std::size_t k=0; k<pose.size(); ++k){
            converted[k] = pose[k] * 3.141f / 180.f;
        }
        return converted;
    }
}