#include <Point2D.h>
#include <Solver.h>
#include <iostream>
using namespace std;

int main() {
	mt::Solver solver(std::make_unique<mt::sample::Point2D>(mt::sample::Box(mt::sample::geometry::Point(-0.1f, -0.1f),
															mt::sample::geometry::Point(1.1f, 1.1f)),
															mt::sample::Box::generateRandomBoxes(3, 20)));

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::Solver::RRTStrategy::Single, mt::Solver::MTStrategy::Serial);

	auto sol = solver.getLastSolution();
	auto tree = solver.getLastTrees();

	return EXIT_SUCCESS;
}