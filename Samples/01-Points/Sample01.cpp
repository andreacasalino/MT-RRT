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
	const size_t Iterations = 1500;
	const mt::Solver::MTStrategy mtStrategy = mt::Solver::MTStrategy::Serial; // use the one you want
	const mt::Solver::RRTStrategy rrtStrategy = mt::Solver::RRTStrategy::Single; // use the one you want 

	mt::Solver solver(std::make_unique<mt::sample::PointProblem>(mt::sample::Obstacle(mt::sample::geometry::Point(-0.1f, -0.1f),
																					  mt::sample::geometry::Point(1.1f, 1.1f)),
																 mt::sample::Obstacle::generateRandomBoxes(5, 30)));

	solver.setMaxIterations(Iterations);

	mt::sample::Results results;

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, rrtStrategy, mtStrategy);
	results.addResult(solver, mtStrategy, rrtStrategy);

	mt::sample::structJSON log;
	log.addElement("problem", static_cast<const mt::sample::PointProblem&>(solver.getProblem()).getJSON());
	log.addElement("results", results.getJSON());

	return EXIT_SUCCESS;
}
