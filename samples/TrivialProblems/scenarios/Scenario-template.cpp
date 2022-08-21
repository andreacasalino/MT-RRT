#include <MT-RRT-core/StandardPlanner.h>

#include <TrivialProblem.h>
#include <TrivialProblemJson.h>

#include <Logger.h>
#include <SampleFramework.h>

class TrivialProblemFramework : public mt_rrt::samples::SampleFramework {
public:
  using mt_rrt::samples::SampleFramework::SampleFramework;

protected:
  std::shared_ptr<mt_rrt::ProblemDescription>
  getProblemDescription_(const nlohmann::json &scene_json) final {
    return mt_rrt::samples::TrivialProblemConverter::CONVERTER.fromJson(
        getSeed(), scene_json);
  }
};

int main(int argc, const char **argv) {
  TrivialProblemFramework framework(SAMPLE_JSON, argc, argv);

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
        mt_rrt::samples::TrivialProblemConverter::CONVERTER, SAMPLE_NAME,
        mt_rrt::utils::make_python_show_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT));

    std::cout << std::endl << std::endl;
  }

  return EXIT_SUCCESS;
}