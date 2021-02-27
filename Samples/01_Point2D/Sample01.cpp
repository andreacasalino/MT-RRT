#include <Point2D.h>
#include <Solver.h>
#include <Logger.h>
using namespace std;

int main() {
	mt::Solver solver(std::make_unique<mt::sample::Point2D>(mt::sample::Box(mt::sample::geometry::Point(-0.1f, -0.1f),
																		    mt::sample::geometry::Point(1.1f, 1.1f)),
															mt::sample::Box::generateRandomBoxes(10, 100)));
	solver.setMaxIterations(500);

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::Solver::RRTStrategy::Bidir, mt::Solver::MTStrategy::Serial);

	mt::sample::Logger log(solver);
	static_cast<const mt::sample::Point2D&>(solver.getProblem()).log(log);
	log.print("Sample01.json");

	return EXIT_SUCCESS;
}