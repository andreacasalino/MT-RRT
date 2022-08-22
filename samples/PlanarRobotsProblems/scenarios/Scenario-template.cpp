#include <MT-RRT-core/StandardPlanner.h>

#include <PlanarRobotsProblem.h>
#include <PlanerRobotsProblemJson.h>

#include <Logger.h>
#include <SampleFramework.h>

class PlanarRobotsProblemFramework : public mt_rrt::samples::SampleFramework {
public:
  using mt_rrt::samples::SampleFramework::SampleFramework;

protected:
  std::shared_ptr<mt_rrt::ProblemDescription>
  getProblemDescription_(const nlohmann::json &scene_json) final {
    return mt_rrt::samples::PlanarRobotsProblemConverter::CONVERTER.fromJson(
        getSeed(), scene_json);
  }

  void setCase(std::pair<mt_rrt::State, mt_rrt::State> &start_end) {
    for (auto &val : start_end.first) {
      val = mt_rrt::samples::to_rad(val);
    }
    for (auto &val : start_end.second) {
      val = mt_rrt::samples::to_rad(val);
    }
  }
};

void interpolate(nlohmann::json &recipient,
                 const std::vector<mt_rrt::State> &sol);

int main(int argc, const char **argv) {
  PlanarRobotsProblemFramework framework(SAMPLE_JSON, argc, argv);

  std::cout << framework << std::endl;

  auto description = framework.getProblemDescription();

  auto parameters = framework.getParameters();

  auto planner = framework.getPlanner(std::move(description));

  for (const auto &[start, end] : framework.getCases()) {
    std::cout << "Trying to connect start point: {" << start
              << "} : with end point: "
              << "{" << end << "}" << std::endl;

    auto solution = planner->solve(start, end, parameters);

    std::cout << "A solution was ";
    if (solution.solution)
      std::cout << "found";
    else
      std::cout << "not found";
    std::cout << std::endl;

    std::cout << "To see the result ";

    // log results
    mt_rrt::utils::log_scenario(
        planner->problem(), solution,
        mt_rrt::samples::PlanarRobotsProblemConverter::CONVERTER, SAMPLE_NAME,
        mt_rrt::utils::make_python_show_sources(
            PLANAR_ROBOTS_PROBLEM_PYTHON_SCRIPT));

    std::cout << std::endl << std::endl;
  }

  return EXIT_SUCCESS;
}
