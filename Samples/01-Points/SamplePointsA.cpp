/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PointProblem.h>
#include <Logger.h>
#include <SampleDescription.h>
using namespace std;

int main() {
	const mt::sample::StrategyType strategyType = mt::sample::StrategyType::MtMultiAgent; // use the one you want
	mt::sample::StrategyParameter parameters;
	parameters.iterations = 1500;
	parameters.steerTrials = 5;

	mt::solver::Solver solver(mt::sample::makeProblemPoint(mt::sample::geometry::Rectangle(mt::sample::geometry::Point(-0.1f, -0.1f), mt::sample::geometry::Point(1.1f, 1.1f))
							  			  			      ,mt::sample::geometry::Rectangle::generateRandomBoxes(5, 30)));

	mt::sample::setStrategy(solver , strategyType, parameters);

	mt::sample::Results results;

	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::solver::RRTStrategy::Single);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Single);
	if(mt::sample::StrategyType::MtMultiAgent != strategyType) {
		// not possible for the multi agent approach
		solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::solver::RRTStrategy::Bidir);
		results.addResult(solver, strategyType, mt::solver::RRTStrategy::Bidir);
	}
	solver.solve({ -0.1f, -0.1f }, {1.1f, 1.1f }, mt::solver::RRTStrategy::Star);
	results.addResult(solver, strategyType, mt::solver::RRTStrategy::Star);

	mt::sample::logResults<mt::sample::SampleDescription<mt::sample::Description>>("Result01.json", solver, results);

	return EXIT_SUCCESS;
}
