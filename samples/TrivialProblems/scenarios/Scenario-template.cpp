#include <TrivialProblem.h>
#include <TrivialProblemJson.h>

#include <SampleFramework.h>

#include <iostream>

class TrivialProblemFramework : public mt_rrt::utils::SampleFramework {
public:
  TrivialProblemFramework(int argc, const char **argv)
      : mt_rrt::utils::SampleFramework(
            argc, argv, mt_rrt::samples::TrivialProblemConverter::CONVERTER) {}

protected:
  std::string defaultJsonConfig() const final { return SAMPLE_JSON; }
};

// see samples/ReadMe.cpp
int main(int argc, const char **argv) {
  // refer to the comment on top of SampleFramework c'tor
  TrivialProblemFramework framework(argc, argv);
  framework.init();

  std::cout << framework << std::endl;

  // ProblemDescription is automatically parsed from config json and the planner
  // is built inside TrivialProblemFramework here we simply access the
  // internally genreated planner
  auto &planner = framework.planner();

  // problems to solve as well as the Parameters to use were parsed from json
  // config
  for (const auto &[start, end] : framework.problems()) {
    std::cout << "Trying to connect start point: {" << start
              << "} : with end point: "
              << "{" << end << "}" << std::endl;

    auto solution = planner.solve(start, end, framework.parameters());

    std::cout << "A solution was ";
    if (solution.solution)
      std::cout << "found";
    else
      std::cout << "not found";
    std::cout << std::endl;

    std::cout << "To see the results ";

    // log results
    mt_rrt::utils::Logger::Log log;
    log.tag = SAMPLE_NAME;
    mt_rrt::samples::TrivialProblemConverter::CONVERTER.toJson2(
        log.content, framework.problemDescription(), solution);
    log.python_visualizer =
        mt_rrt::utils::default_python_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT);
    mt_rrt::utils::Logger::log(log);

    std::cout << std::endl << std::endl;
  }

  return EXIT_SUCCESS;
}
