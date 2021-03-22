/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>
#include <Logger.h>
#include <SampleDescription.h>
using namespace std;

int main() {
	size_t Iterations = 3000;

	mt::ProblemPtr problem;
	mt::NodeState start, target;
	{
		auto imported = mt::sample::importNavigationProblem(std::string(CONFIG_FOLDER) + "/Sample01-config");
		problem = std::move(std::get<0>(imported));
		start = std::get<1>(imported);
		target = std::get<2>(imported);
	}
	mt::solver::Solver solver(std::move(problem));

	solver.setThreadAvailability(0);
	solver.setSteerTrials(15);
	solver.saveTreesAfterSolve();

	mt::sample::Results results(solver, start, target, 0, true);

	mt::sample::structJSON log;
	solver.useProblem([&log](const mt::Problem& problem){
		log.addElement("problem", dynamic_cast<const mt::sample::SampleDescription<mt::sample::Description>*>(problem.getTrajManager())->logDescription());
	});
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Result02.json");

	return EXIT_SUCCESS;
}
