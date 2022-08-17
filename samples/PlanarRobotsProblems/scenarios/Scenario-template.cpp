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

  nlohmann::json scenario_json;
  mt_rrt::utils::from_file(scenario_json, SAMPLE_JSON);

  mt_rrt::State start, end;
  auto static_data_problem =
      mt_rrt::samples::make_problem_description(0, scenario_json, start, end);

  // define some default parameters
  auto parameters = mt_rrt::Parameters{
      mt_rrt::ExpansionStrategy::Star, mt_rrt::SteerIterations{10},
      mt_rrt::Iterations{3000}, mt_rrt::Determinism{0.1f}, true};
  // override the parameters according to the scenario default
  // specifications
  mt_rrt::samples::from_json(scenario_json, parameters);
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

// return a + c(b-a)
mt_rrt::State linear_combination(const mt_rrt::State &a, const mt_rrt::State &b,
                                 float c) {
  mt_rrt::State result = a;
  for (std::size_t k = 0; k < a.size(); ++k) {
    result[k] += c * (b[k] - a[k]);
  }
  return result;
}

static const float MAX_JOINT_DISTANCE = mt_rrt::samples::to_rad(15);

std::vector<mt_rrt::State> interpolate(const mt_rrt::State &start,
                                       const mt_rrt::State &end) {
  std::size_t intervals = static_cast<std::size_t>(
      std::ceil(mt_rrt::samples::euclidean_distance(start.data(), end.data(),
                                                    start.size()) /
                MAX_JOINT_DISTANCE));
  intervals = std::max<std::size_t>(1, intervals);
  const float delta_c = 1.f / static_cast<float>(intervals);
  float c = delta_c;
  std::vector<mt_rrt::State> result;
  for (std::size_t i = 1; i < intervals; ++i) {
    result.emplace_back(linear_combination(start, end, c));
    c += delta_c;
  }
  result.push_back(end);
  return result;
}

// interpolate the solution, mostly to generate a cool plot visualizing results
void interpolate(nlohmann::json &recipient,
                 const std::vector<mt_rrt::State> &sol) {
  std::vector<mt_rrt::State> interpolated;
  interpolated.push_back(sol.front());
  for (std::size_t k = 1; k < sol.size(); ++k) {
    auto interpolated_segment = interpolate(sol[k - 1], sol[k]);
    interpolated.insert(interpolated.end(), interpolated_segment.begin(),
                        interpolated_segment.end());
  }
  recipient = interpolated;
}
