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
	size_t Iterations = 1500;

	mt::Solver solver(std::make_unique<mt::sample::PointProblem>(mt::sample::Obstacle(mt::sample::geometry::Point(-0.1f, -0.1f),
																					  mt::sample::geometry::Point(1.1f, 1.1f)),
																 mt::sample::Obstacle::generateRandomBoxes(10, 100)));

	solver.setMaxIterations(Iterations);
	solver.setSteerTrials(5);

	mt::sample::Results results(solver, { -0.1f, -0.1f }, { 1.1f, 1.1f }, 0);

	mt::sample::structJSON log;
	log.addElement("problem", static_cast<const mt::sample::PointProblem&>(solver.getProblem()).getJSON());
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Sample-cluttered.json");

	return EXIT_SUCCESS;
}
