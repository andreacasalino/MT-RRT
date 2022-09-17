#include <MT-RRT-core/Planner.h>

#include <MT-RRT-core/StandardPlanner.h>

void main() {
	// First of all, you need to explain how the particular class of problem(s) you want to solve is defined.
	// This is done by deriving an object from mt_rrt::Connector. This object contains the knowledge of the problem and is
	// foundamental to let MT-RRT extend the search tree(s). Let's say you have derived a concrete connector named MyConnector
	mt_rrt::ConnectorPtr my_connector = std::make_unique<MyConnector>(...);

	// secondly, you need to define the object handling the sampling of states, deriving something from the interface mt_rrt::Sampler.
	// In a certain sense, also this object stores a kind of knowledge of the particular problem you wan to solve.
	// Most of the time, you can use the already defined concrete sampler named mt_rrt::HyperBox.
	// Let's say your concrete sampler is called MySampler
	mt_rrt::SamplerPtr my_sampler = std::make_unique<MySampler>(...);

	// group the above infomration into the structure called mt_rrt::ProblemDescription
	mt_rrt::ProblemDescription problem_description;
	problem_description.connector = std::move(my_connector);
	problem_description.sampler = std::move(my_sampler);
	problem_description.gamma.set(...); // the parameters regulating RRT* strategy, refer to the documentation at Section "Compute the optimal solution: the RRT*"

	// You are now ready to build a planner.
	// This planner will absorb the problem description and will be able to solve 1 or multiple problems of the same kind
	// The easiest planner you can build is mt_rrt::StandardPlanner, implementing the single threaded classic strategies of the RRT.
	// More advanced multi threaded planners are instead contained in src/multi-threaded
	mt_rrt::StandardPlanner planner(std::move(problem_description));

	{
		// solve the first problem
		// define the states to connect
		mt_rrt::State start_state = ...;
		mt_rrt::State end_state = ...;

		// define the parameters
		mt_rrt::Parameters parameters;
		parameters.iterations.set(1500); // number of maximum iterations
		parameters.expansion_strategy = mt_rrt::ExpansionStrategy::Star; // set the strategy, see Section "Background on RRT" of the documentation
		// ... and some others

		// try to solve the problem
		mt_rrt::PlannerSolution solution = planner.solve(start_state, end_state, parameters);
		std::vector<mt_rrt::State> found_sequence = solution.solution.value(); // access the found solution (is nullopt if no solution was found)
		auto time = solution.time; // access the time spent by the planner to find a solution
		// ... you can access in that structure also some other info
	}

	{
		// solve another problem ... you don't need to build a new planner if the problem is of the same kind.
		// Indeed, you can re-use the same planner, but passing the info of this other particular problem to solve
		// IMPORTANT !! Clearly, in order to avoid data race, you can solve only 1 problem at a time per planner
		mt_rrt::State start_state = ...;
		mt_rrt::State end_state = ...;
		mt_rrt::Parameters parameters = ...;
		mt_rrt::PlannerSolution solution = planner.solve(start_state, end_state, parameters);
	}
}
