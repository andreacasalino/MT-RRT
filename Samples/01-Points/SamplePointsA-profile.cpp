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
	mt::sample::StrategyParameter parameters;
	parameters.iterations = 1500;
	parameters.steerTrials = 5;

	mt::solver::Solver solver(mt::sample::makeProblemPoint(mt::sample::geometry::Rectangle(mt::sample::geometry::Point(-0.1f, -0.1f), mt::sample::geometry::Point(1.1f, 1.1f))
							  			  			      ,mt::sample::geometry::Rectangle::generateRandomBoxes(10, 100)));

	mt::sample::Results results(solver, { -0.1f, -0.1f }, { 1.1f, 1.1f }, parameters);

	mt::sample::logResults<mt::sample::SampleDescription<mt::sample::Description>>("Result02.json", solver, results);

	return EXIT_SUCCESS;
}
