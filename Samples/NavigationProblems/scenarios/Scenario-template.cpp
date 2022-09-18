#include <NavigationProblem.h>
#include <NavigationProblemJson.h>

#include <SampleFramework.h>

#include <iostream>

class NavigationProblemFramework : public mt_rrt::utils::SampleFramework {
public:
    NavigationProblemFramework(int argc, const char** argv) :
        mt_rrt::utils::SampleFramework(argc, argv, mt_rrt::samples::NavigationProblemConverter::CONVERTER) {}

protected:
    std::string defaultJsonConfig() const final {
        return SAMPLE_JSON;
    }
};

mt_rrt::State convert_state(const mt_rrt::State &subject) {
  auto result = subject;
  result[2] = mt_rrt::utils::to_rad(result[2]);
  return result;
}

// see samples/ReadMe.cpp
int main(int argc, const char **argv) {
  NavigationProblemFramework framework(argc, argv);
  framework.init();

  std::cout << framework << std::endl;

  // ProblemDescription is automatically parsed from config json and the planner is built inside TrivialProblemFramework
  // here we simply access the internally genreated planner
  auto& planner = framework.planner();

  for (const auto& [start, end] : framework.problems()) {
    std::cout << "Trying to connect start point: {" << start
              << "} : with end point: "
              << "{" << end << "}" << std::endl;

    auto solution =
        planner.solve(convert_state(start), convert_state(end), framework.parameters());

    std::cout << "A solution was ";
    if (solution.solution)
      std::cout << "found";
    else
      std::cout << "not found";
    std::cout << std::endl;

    std::cout << "To see the result ";

    // log results
    mt_rrt::utils::Logger::Log log;
    log.tag = SAMPLE_NAME;
    mt_rrt::samples::NavigationProblemConverter::CONVERTER.toJson2(log.content, framework.problemDescription(), solution);
    log.python_visualizer = mt_rrt::utils::default_python_sources(NAVIGATION_PROBLEM_PYTHON_SCRIPT);
    mt_rrt::utils::Logger::log(log);

    std::cout << std::endl << std::endl;
  }

  return EXIT_SUCCESS;
}
