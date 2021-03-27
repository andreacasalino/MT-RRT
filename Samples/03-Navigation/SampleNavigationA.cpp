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
	const mt::sample::StrategyType strategyType = mt::sample::StrategyType::Serial; // use the one you want
	mt::sample::StrategyParameter parameters;
	parameters.iterations = 3000;
	parameters.steerTrials = 15;

	mt::ProblemPtr problem;
	mt::NodeState start, target;
	{
		auto imported = mt::sample::importNavigationProblem(std::string(CONFIG_FOLDER) + "/ConfigA-debug");
		problem = std::move(std::get<0>(imported));
		start = std::get<1>(imported);
		target = std::get<2>(imported);
	}
	mt::solver::Solver solver(std::move(problem));

	mt::sample::setStrategy(solver , strategyType, parameters);

	mt::sample::Results results;

	solver.solve(start, target, mt::solver::RRTStrategy::Single);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Single);
	if(mt::sample::StrategyType::MtMultiAgent != strategyType) {
		// not possible for the multi agent approach
		solver.solve(start, target, mt::solver::RRTStrategy::Bidir);
		results.addResult(solver, strategyType, mt::solver::RRTStrategy::Bidir);
	}
	solver.solve(start, target, mt::solver::RRTStrategy::Star);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Star);

	mt::sample::logResults<mt::sample::SampleDescription<mt::sample::Description>>("Result01.json", solver, results);

	return EXIT_SUCCESS;
}
