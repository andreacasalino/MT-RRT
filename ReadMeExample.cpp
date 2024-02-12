#include <MT-RRT/Planner.h>

#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/StandardPlanner.h>

void main() {
  // First of all, you need to explain to MT-RRT how the particular class of
  // problem(s) you want to solve is made.
  // This is done by defining the specific conenctor, i.e. an object extending
  // mt_rrt::Connector, which will contain the knowledge of the problem and is
  // foundamental to let MT-RRT extend the search tree(s). Examples of
  // connectors can be found in the samples folder:
  //		-
  //[TrivialProblem](./problems/problem-trivial/header/TrivialProblem.h), check
  // also `Planar maze problem` Section of the documentation
  //		-
  //[PlanarRobotsProblem](./problems/problem-planarRobots/header/PlanarRobotsProblem.h),
  // check
  // also `Articulated arm problem` Section of the documentation
  //		-
  //[NavigationProblem](./problems/problem-navigation/header/NavigationProblem.h),
  // check
  // also `Navigation problem` Section of the documentation
  //
  //
  // Let's assume you defined a connector named MyConnector
  mt_rrt::ConnectorPtr my_connector = std::make_unique<MyConnector>(...);

  // secondly, you need to define a sampler, i.e. an object extending
  // mt_rrt::Sampler. This object is used to sample new random states toward
  // which the trees will be extended in the attempt explore the state space and
  // find a solution that connects the starting and ending state you want to
  // connect. In a certain sense, also this object stores some knowledge of the
  // particular problem you want to solve. Most of the time, you can use the
  // ready to use sampler named mt_rrt::HyperBox. Let's say your concrete
  // sampler is called MySampler
  mt_rrt::SamplerPtr my_sampler = std::make_unique<MySampler>(...);

  // The abvoe pieces of information has to be grouped to a structure called
  // mt_rrt::ProblemDescription
  mt_rrt::ProblemDescription problem_description;
  problem_description.simmetry =
      true; // true / false, if the problem is simmetric or not, i.e. the
            // trajectory conencting A to B can be trivially inverted to get the
            // one going from B to A
  problem_description.gamma.set(
      ...); // the parameters regulating the RRT* strategy, refer to Section
            // "Compute the optimal solution: the RRT*" of the documentation
  problem_description.connector = std::move(my_connector);
  problem_description.sampler = std::move(my_sampler);

  // You are now ready to build a planner.
  // This planner will absorb the problem description and will be able to solve
  // 1 or many problems of the kind represented by the above parameters. The
  // easiest planner you can build is mt_rrt::StandardPlanner, implementing the
  // single threaded classic RRT. More advanced multi threaded
  // planners can be found [here](src/multi-threaded/header/MT-RRT), check also
  // the Documentation.
  mt_rrt::StandardPlanner planner(std::move(problem_description));

  {
    // solve the first problem
    // define the states to connect
    std::vector<float> start_state = ...;
    std::vector<float> end_state = ...;

    // define the parameters
    mt_rrt::Parameters parameters;
    parameters.iterations.set(1500); // number of maximum iterations
    parameters.expansion_strategy =
        mt_rrt::ExpansionStrategy::Star; // set the strategy, see Section
                                         // "Background on RRT" of the
                                         // documentation
    // ... and the others, check the mt_rrt::Parameters definition

    // solve the problem
    mt_rrt::PlannerSolution solution =
        planner.solve(start_state, end_state, parameters);
    std::vector<std::vector<float>> found_sequence =
        solution.solution; // access the found solution (is an empty vector if
                           // no solution was found)
    auto time =
        solution
            .time; // access the time spent by the planner to find a solution
                   // ... you can access in that structure also some other info
  }

  {
    // solve another problem ... you don't need to build a new planner if the
    // problem is of the same kind. Indeed, you can re-use the same planner, but
    // passing the info of this other particular problem to solve.
    //
    // IMPORTANT !!! Clearly, in order to avoid data race, you can solve only 1
    // problem at a time per planner
    std::vector<float> start_state = ...;
    std::vector<float> end_state = ...;
    mt_rrt::Parameters parameters = ...;
    mt_rrt::PlannerSolution solution =
        planner.solve(start_state, end_state, parameters);
  }
}
