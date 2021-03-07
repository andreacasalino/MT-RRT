/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ManipulatorProblem.h>
#include <Logger.h>
using namespace std;

int main() {
	size_t Iterations = 3500;

    std::vector<mt::sample::Manipulator> robots = { mt::sample::make_manipulator({282, 336, 136.82113524097718, 124.80785231707178, 13, 6}),
                                                    mt::sample::make_manipulator({616, 341, 105.4387124482674, 117.65202930676547, 11, 5}) };
    std::vector<mt::sample::Sphere> obstacles = { mt::sample::Sphere(39, 315, 78), mt::sample::Sphere(879, 318, 97) };

	mt::Solver solver(std::make_unique<mt::sample::ManipulatorProblem>(robots, obstacles));

	solver.setMaxIterations(Iterations);
	solver.setSteerTrials(5);

	mt::sample::Results results(solver, mt::sample::degree2rad(mt::NodeState(-61.15898683560858, 23.646343347177787, 120.17635342123464, 40.46478809083812))
                                      , mt::sample::degree2rad(mt::NodeState(55.41244173582004, -24.353656652822217, 180, -14.39235476630474)), 0, true);

	mt::sample::structJSON log;
	log.addElement("problem", static_cast<const mt::sample::ManipulatorProblem&>(solver.getProblem()).getJSON());
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Sample-multi-arm.json");

	return EXIT_SUCCESS;
}
