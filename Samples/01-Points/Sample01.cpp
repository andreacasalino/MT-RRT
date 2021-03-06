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
	const size_t Iterations = 3000;
	const mt::Solver::MTStrategy mtStrategy = mt::Solver::MTStrategy::Serial; // use the one you want

	mt::Solver solver(std::make_unique<mt::sample::PointProblem>(mt::sample::Obstacle(mt::sample::geometry::Point(-0.1f, -0.1f),
																					  mt::sample::geometry::Point(1.1f, 1.1f)),
																 mt::sample::Obstacle::generateRandomBoxes(5, 30)));

	solver.setMaxIterations(Iterations);
	solver.setThreadAvailability(3); // ignored when using serial approach

	mt::sample::Results results;

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::Solver::RRTStrategy::Single, mtStrategy);
	results.addResult(solver, mtStrategy, mt::Solver::RRTStrategy::Single);

	solver.solve({ -0.1f, -0.1f }, { 1.1f, 1.1f }, mt::Solver::RRTStrategy::Bidir, mtStrategy);
	results.addResult(solver, mtStrategy, mt::Solver::RRTStrategy::Bidir);

	solver.solve({ -0.1f, -0.1f }, { 1.1f, 1.1f }, mt::Solver::RRTStrategy::Star, mtStrategy);
	results.addResult(solver, mtStrategy, mt::Solver::RRTStrategy::Star);

	mt::sample::structJSON log;
	log.addElement("problem", static_cast<const mt::sample::PointProblem&>(solver.getProblem()).getJSON());
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Sample01.json");

	return EXIT_SUCCESS;
}
