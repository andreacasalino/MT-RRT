/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PointProblem.h>
#include <Logger.h>
using namespace std;

int main() {
	const size_t Iterations = 2000;

	mt::solver::Solver solver(std::make_unique<mt::sample::PointProblem>(mt::sample::Obstacle(mt::sample::geometry::Point(-0.1f, -0.1f), mt::sample::geometry::Point(1.1f, 1.1f)),
																 		 mt::sample::Obstacle::generateRandomBoxes(10, 100)
																		));

	solver.setThreadAvailability(0);
	solver.setSteerTrials(5);
	solver.saveTreesAfterSolve();

	mt::sample::Results results(solver, { -0.1f, -0.1f }, { 1.1f, 1.1f }, 0);

	mt::sample::structJSON log;
	solver.useProblem([&log](const mt::Problem& problem){
		log.addElement("problem", static_cast<const mt::sample::PointProblem&>(problem).getJSON());
	});
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Result02.json");

	return EXIT_SUCCESS;
}
