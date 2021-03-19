/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>
#include <sampler/HyperBox.h>
#include "CartTrajectory.h"
#include <Error.h>
#include <Importer.h>

namespace mt::sample {
    sampling::SamplerPtr make_sampler(const geometry::Rectangle& boundaries) {
        NodeState low = { boundaries.getXMin() , boundaries.getYMin(), -3.141f };
        NodeState upp = { boundaries.getXMax() , boundaries.getYMax(),  3.141f };
        return std::make_unique<sampling::HyperBox>(low, upp);
    }

    inline float convert(const std::string& buff) {
        return static_cast<float>(std::atof(buff.c_str()));
    }

    Description make_data(const geometry::Rectangle& boundaries, const std::vector<geometry::Sphere>& obstacles, const Cart& cart, const float& blend) {
        return Description{boundaries, obstacles, cart, blend};
    }

    std::tuple<Description, NodeState, NodeState> importProblem(const std::string& configFileName) {
        Importer importer(configFileName, "bound", "cart", "start", "target");

        auto boundRaw = importer.find("bound");
        geometry::Rectangle boundaries( geometry::Point((*boundRaw[0])[0], (*boundRaw[0])[1]), geometry::Point((*boundRaw[0])[2], (*boundRaw[0])[3]) );
        std::vector<geometry::Sphere> obstacles;
        {
            auto obstaclesRaw = importer.find("obstacle");
            obstacles.reserve(obstaclesRaw.size());
            for(auto it =obstaclesRaw.begin(); it!=obstaclesRaw.end(); ++it) {
                if(3 != (*it)->size()) {
                    throw Error("invalid obstacle");
                }
                obstacles.emplace_back(geometry::Sphere((**it)[0], (**it)[1], (**it)[2]));
            }
        }

        auto cartRaw = importer.find("cart");
        Cart cart = Cart{Positive<float>((*cartRaw[0])[0]) , Positive<float>((*cartRaw[0])[1]) };
        float radius = (*cartRaw[0])[2];

        NodeState start = *importer.find("start").front();
        if(3 != start.size()) {
            throw Error("inconsistent state size");
        }
        NodeState target = *importer.find("target").front();
        if(3 != target.size()) {
            throw Error("inconsistent state size");
        }

        return std::make_tuple(make_data(boundaries, obstacles, cart, radius), start, target);
    }
}
