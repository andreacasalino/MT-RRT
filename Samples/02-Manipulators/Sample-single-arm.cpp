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

    std::vector<mt::sample::Manipulator> robots = { mt::sample::make_manipulator({663, 311, 127.40260206137343, 181.0331461362808, 122.18428704215611, 120.61509026651682, 27, 21, 12, 6}) };
    std::vector<mt::sample::Sphere> obstacles = { mt::sample::Sphere(752, 47, 44) };

	mt::Solver solver(std::make_unique<mt::sample::ManipulatorProblem>(robots, obstacles));

	solver.setMaxIterations(Iterations);
	solver.setSteerTrials(5);

	mt::sample::Results results(solver, mt::sample::degree2rad(mt::NodeState(-118.60697018212753, -50.89016996706415, -21.35312568492888, -29.44206799448135 ))
                                      , mt::sample::degree2rad(mt::NodeState(-1.1703056757570762, -50.89016996706415, -21.35312568492888, -29.44206799448135 )), 0, true);

	mt::sample::structJSON log;
	log.addElement("problem", static_cast<const mt::sample::ManipulatorProblem&>(solver.getProblem()).getJSON());
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Sample02.json");

	return EXIT_SUCCESS;
}
