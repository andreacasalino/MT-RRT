/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblemJson.h>

#include <JsonConvert.h>

namespace mt_rrt::samples {
    void to_json(nlohmann::json& j, const Cart& subject) {
        j["width"] = subject.getWidth();
        j["length"] = subject.getLength();
    }

    void to_json(nlohmann::json& j, const Sphere& subject) {
        j["ray"] = subject.ray.get();
        j["center"] = subject.center;
    }

    void from_json(const nlohmann::json& j, std::vector<Sphere>& obstacles) {
        for (const auto& s : j) {
            auto& sphere = obstacles.emplace_back();
            float ray = s["ray"];
            sphere.ray.set(ray);
            Point center = s["center"];
            sphere.center = center;
        }
    }
} // namespace mt_rrt::samples
