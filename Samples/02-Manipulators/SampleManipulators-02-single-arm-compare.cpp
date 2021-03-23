/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ManipulatorProblem.h>
#include <Logger.h>
#include <SampleDescription.h>
using namespace std;

int main() {
	mt::sample::StrategyParameter parameters;
	parameters.iterations = 2000;
	parameters.steerTrials = 10;

	mt::ProblemPtr problem;
	mt::NodeState start, target;
	{
		auto imported = mt::sample::importManipulatorProblem(std::string(CONFIG_FOLDER) + "/Sample01-config");
		problem = std::move(std::get<0>(imported));
		start = mt::sample::degree2rad(std::get<1>(imported));
		target = mt::sample::degree2rad(std::get<2>(imported));
	}
	mt::solver::Solver solver(std::move(problem));

	mt::sample::Results results(solver, start, target, parameters);

	mt::sample::logResults<mt::sample::SampleDescription<mt::sample::Description>>("Result02.json", solver, results);

	return EXIT_SUCCESS;
}
