#include <SampleBase.h>
#include <TrivialProblem.h>
#include <TrivialProblemConversions.h>

#include <list>

namespace mt_rrt {
std::vector<geom::Box> from_json_boxes(const nlohmann::json &obstacles) {
  std::list<geom::Box> boxes;
  std::unordered_map<std::string, const geom::Box *> labelled_boxes;

  std::list<const nlohmann::json *> open;
  for (const auto &box_json : obstacles) {
    if (box_json.contains("copy")) {
      open.push_back(&box_json);
      continue;
    }
    std::optional<geom::Transform> trsf;
    if (box_json.contains("transform")) {
      trsf = from_json_transform(box_json["transform"]);
    }
    auto &added = boxes.emplace_back(from_json_point(box_json["min"]),
                                     from_json_point(box_json["max"]), trsf);
    if (box_json.contains("label")) {
      labelled_boxes[box_json["label"]] = &added;
    }
  }

  while (!open.empty()) {
    bool blocked = true;
    auto it = open.begin();
    while (it != open.end()) {
      const auto &box_json = **it;
      std::string copy_from = box_json["copy"];
      auto it_box = labelled_boxes.find(copy_from);
      if (it_box == labelled_boxes.end()) {
        ++it;
        continue;
      }

      blocked = false;
      auto &added = boxes.emplace_back(*it_box->second);
      if (box_json.contains("transform")) {
        if (added.trsf.has_value()) {
          added.trsf = geom::Transform::combine(
              from_json_transform(box_json["transform"]), added.trsf.value());
        } else {
          added.trsf = from_json_transform(box_json["transform"]);
        }
      }
      if (box_json.contains("label")) {
        labelled_boxes[box_json["label"]] = &added;
      }
      it = open.erase(it);
    }

    if (blocked) {
      throw Error{"Invalid obstacles"};
    }
  }

  return std::vector<geom::Box>{boxes.begin(), boxes.end()};
}

template <>
ProblemDescription
from_json<trivial::TrivialProblemConnector>(const nlohmann::json &src) {
  unsigned seed = src["seed"];
  geom::Boxes boxes = from_json_boxes(src["obstacles"]);
  return std::move(*trivial::TrivialProblemConnector::make(Seed{seed}, boxes));
}
} // namespace mt_rrt

int main(int argc, const char **argv) {
  mt_rrt::SampleBase<mt_rrt::trivial::TrivialProblemConnector> sample{
      argc, argv, "TrivialProblem"};

  sample.process();

  return EXIT_SUCCESS;
}
