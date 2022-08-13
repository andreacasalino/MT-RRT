////////////////////////////////////////////////////////////////
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
// !!             have a look to ../ReadMe.md              !! //
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
////////////////////////////////////////////////////////////////

// the class problem is defined here
#include <PlanarRobotsProblem.h>

#include <MT-RRT-core/StandardPlanner.h>

// functionalities to parse the input from the user and generate:
//  - a valid planner
//  - the parameters used to configure such a planner
#include <Configurations.h>
// just a bunch of utilities for generating log files
#include <Logger.h>
#include <PlanerRobotsProblemIO.h>

#include <algorithm>
#include <iostream>

std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject);

void interpolate(nlohmann::json &recipient,
                 const std::vector<mt_rrt::State> &sol);

int main(int argc, const char **argv) {
  // parse user defined inputs
  nlohmann::json user_configs;
  mt_rrt::samples::to_json(user_configs, argc, argv);

  mt_rrt::State start, end;
  auto static_data_problem = mt_rrt::samples::make_problem_description(
      0, std::string{SAMPLE_JSON}, start, end);

  // define some default parameters
  auto parameters = mt_rrt::Parameters{
      mt_rrt::ExpansionStrategy::Bidir, mt_rrt::SteerIterations{10},
      mt_rrt::Iterations{3000}, mt_rrt::Determinism{0.1f}, true};
  // override the parameters according to the user specs
  mt_rrt::samples::from_json(user_configs, parameters);

  // build the kind of planner specified by the user
  auto planner = mt_rrt::samples::from_json(
      user_configs,
      mt_rrt::ProblemDescription{
          std::move(static_data_problem->sampler),
          std::move(static_data_problem->connector), true,
          mt_rrt::Positive<float>{static_data_problem->gamma.get()}});

  /////////////////////////////////////////////////////////////////////////
  // find solution to the first problem using the generated planner
  /////////////////////////////////////////////////////////////////////////
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
  mt_rrt::utils::log_scenario(planner->problem(), solution,
                              mt_rrt::samples::PosesConnectorLogger::LOGGER,
                              interpolate, SAMPLE_NAME,
                              mt_rrt::utils::make_python_show_sources(
                                  PLANAR_ROBOTS_PROBLEM_PYTHON_SCRIPT));
  std::cout << std::endl << std::endl;

  return EXIT_SUCCESS;
}

std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject) {
  for (const auto val : subject) {
    s << ' ' << val;
  }
  return s;
}

// interpolate the solution, mostly to generate a cool plot visualizing results
void interpolate(nlohmann::json &recipient,
                 const std::vector<mt_rrt::State> &sol) {
  std::vector<mt_rrt::State> interpolated;
  interpolated.push_back(sol.front());
  const std::size_t state_size = interpolated.back().size();
  for (std::size_t k = 1; k < state_size; ++k) {
    std::size_t intervals = static_cast<std::size_t>(
        std::ceil(mt_rrt::samples::euclidean_distance(
                      sol[k - 1].data(), sol[k].data(), state_size) /
                  static_cast<float>(0.1f)));
    intervals = std::max<std::size_t>(1, intervals);
    const float delta_advancement = 1.f / static_cast<float>(intervals);
    const float delta_complement = 1.f - delta_advancement;
    for (std::size_t i = 1; i < intervals; ++i) {
      mt_rrt::State state;
      for (std::size_t pos = 0; pos < state_size; ++pos) {
        state.push_back(interpolated.back()[pos] * delta_complement +
                        sol[k][pos] * delta_advancement);
      }
      interpolated.emplace_back(std::move(state));
    }
    interpolated.push_back(sol[k]);
  }
  recipient = interpolated;
}
