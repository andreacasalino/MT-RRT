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
#include <fstream>

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
        std::ifstream f(configFileName);
        if(!f.is_open()) {
            throw Error("invalid config file");
        }

        auto trimmer = [](const std::string& line) -> std::list<std::string> {
            std::istringstream iss(line);
            std::list<std::string> slices;
            while (!iss.eof()) {
                slices.emplace_back(std::string());
                iss >> slices.back();
                if(slices.back().empty()) slices.pop_back();
            }
            return slices;
        };

        std::unique_ptr<geometry::Rectangle> boundaries;
        std::vector<geometry::Sphere> obstacles;
        std::unique_ptr<Cart> cart;
        std::unique_ptr<float> radius;
        NodeState start;
        NodeState target;

        std::string line;
        std::list<std::string> slices;
        while (!f.eof()) {
            std::getline(f, line);
            slices = trimmer(line);

            if(!slices.empty()) {
                if(0 == slices.front().compare("obstacle")) {
                    slices.pop_front();
                    float coor[3];
                    coor[0] = convert(slices.front());
                    slices.pop_front();
                    coor[1] = convert(slices.front());
                    slices.pop_front();
                    coor[2] = convert(slices.front());
                    slices.pop_front();
                    obstacles.push_back(geometry::Sphere(coor[0], coor[1], coor[2]));
                }
                else if(0 == slices.front().compare("bound")) {
                    slices.pop_front();
                    if(slices.size() != 4) {
                        throw Error("invalid boudnaries");
                    }                    
                    float x_min = convert(slices.front());
                    slices.pop_front();
                    float y_min = convert(slices.front());
                    slices.pop_front();  
                    float x_max = convert(slices.front());
                    slices.pop_front();
                    float y_max = convert(slices.front());
                    slices.pop_front();
                    boundaries = std::make_unique<geometry::Rectangle>(geometry::Point(x_min, y_min),  geometry::Point(x_max, y_max));
                }
                else if(0 == slices.front().compare("cart")) {
                    slices.pop_front();
                    if(slices.size() != 3) {
                        throw Error("invalid cart data");
                    }                    
                    float w = convert(slices.front());
                    slices.pop_front();
                    float h = convert(slices.front());
                    slices.pop_front();
                    cart = std::make_unique<Cart>(Positive<float>(w), Positive<float>(h));
                    radius = std::make_unique<float>(convert(slices.front()));
                }
                else if(0 == slices.front().compare("start")) {
                    slices.pop_front();
                    if(!start.empty()) {
                        throw Error("multiple start configurations found");
                    }
                    while (!slices.empty()) {
                        start.push_back(convert(slices.front()));
                        slices.pop_front();
                    }
                }
                else if(0 == slices.front().compare("target")) {
                    slices.pop_front();
                    if(!target.empty()) {
                        throw Error("multiple target configurations found");
                    }
                    while (!slices.empty()) {
                        target.push_back(convert(slices.front()));
                        slices.pop_front();
                    }
                }
            }
        }

        if(nullptr == boundaries) {
            throw Error("boudnaries not specified");
        }
        if(nullptr == cart) {
            throw Error("cart data not specified");
        }
        if(nullptr == radius) {
            throw Error("blending radius not specified");
        }
        if(3 != start.size()) {
            throw Error("inconsistent state size");
        }
        if(3 != target.size()) {
            throw Error("inconsistent state size");
        }
        return std::make_tuple(make_data(*boundaries, obstacles, *cart, *radius), start, target);
    }
}
