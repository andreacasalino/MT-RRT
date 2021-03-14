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
	const mt::sample::StrategyType strategyType = mt::sample::StrategyType::Serial; // use the one you want

	mt::ProblemPtr problem;
	mt::NodeState start, target;
	{
		auto imported = mt::sample::importProblem(std::string(CONFIG_FOLDER) + "/Sample03-config");
		problem = std::move(std::get<0>(imported));
		start = mt::sample::degree2rad(std::get<1>(imported));
		target = mt::sample::degree2rad(std::get<2>(imported));
	}
	mt::solver::Solver solver(std::move(problem));

	auto strategy = mt::sample::make_strategy(strategyType);
	strategy->getIterationsMax().set(Iterations);
	solver.setStrategy(std::move(strategy));
	solver.setThreadAvailability(0);
	solver.setSteerTrials(5);

	mt::sample::Results results;

	solver.solve(start, target, mt::solver::RRTStrategy::Single);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Single);

	try {
		// not possible for the multi agent approach
		solver.solve(start, target, mt::solver::RRTStrategy::Bidir);
		results.addResult(solver, strategyType, mt::solver::RRTStrategy::Bidir);
	}
	catch(...) {
	}

	solver.solve(start, target, mt::solver::RRTStrategy::Star);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Star);

	mt::sample::structJSON log;
	solver.useProblem([&log](const mt::Problem& problem){
		log.addElement("problem", static_cast<const mt::sample::ManipulatorProblem&>(problem).getJSON());
	});
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Result03.json");

	return EXIT_SUCCESS;
}