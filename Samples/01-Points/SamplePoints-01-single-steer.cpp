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
	const size_t Iterations = 2000;
	const mt::sample::StrategyType strategyType = mt::sample::StrategyType::MtLinkedTrees; // use the one you want

	mt::solver::Solver solver(std::make_unique<mt::sample::PointProblem>(mt::sample::Obstacle(mt::sample::geometry::Point(-0.1f, -0.1f), mt::sample::geometry::Point(1.1f, 1.1f)),
																 		 mt::sample::Obstacle::generateRandomBoxes(5, 30)
																		));

	auto strategy = mt::sample::make_strategy(strategyType);
	strategy->getIterationsMax().set(Iterations);
	solver.setStrategy(std::move(strategy));
	solver.setThreadAvailability(0);

	mt::sample::Results results;

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::solver::RRTStrategy::Single);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Single);

	try {
		// not possible for the multi agent approach
		solver.solve({ -0.1f, -0.1f }, { 1.1f, 1.1f }, mt::solver::RRTStrategy::Bidir);
		results.addResult(solver, strategyType, mt::solver::RRTStrategy::Bidir);
	}
	catch(...) {
	}

	solver.solve({ -0.1f, -0.1f }, { 1.1f, 1.1f }, mt::solver::RRTStrategy::Star);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Star);

	mt::sample::structJSON log;
	solver.useProblem([&log](const mt::Problem& problem){
		log.addElement("problem", static_cast<const mt::sample::PointProblem&>(problem).getJSON());
	});
	log.addEndl();
	log.addElement("results", results.getJSON());
	printData(log, "Result01.json");

	return EXIT_SUCCESS;
}
