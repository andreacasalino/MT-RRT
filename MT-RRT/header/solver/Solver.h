/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SOLVER_H
#define MT_RRT_SOLVER_H

#include <Problem.h>
#include <Tree.h>
#include <mutex>
#include <chrono>

namespace mt::solver {
	enum RRTStrategy { Single, Bidir, Star };

	class Strategy;

	struct SolutionInfo {
		std::chrono::milliseconds		time = std::chrono::milliseconds(0);
		std::size_t						iterations = 0;
		std::vector<NodeState>			solution;
		std::vector<TreePtr>			trees;
	};

	struct SolverData {
		std::mutex solverMutex;
		std::vector<ProblemPtr> problemsBattery;
		bool saveComputedTrees = false;
	};
	
	/** \brief Interface for a planner.
	\details For solving a planning problem you must first build the solver, using 
	I_Planner::Get_canonical, I_Planner::Get_query___parall, I_Planner::Get_shared__parall, I_Planner::Get_copied__parall or I_Planner::Get_multi_ag_parall. 
	Then you can call I_Planner::RRT_basic, I_Planner::RRT_bidirectional or I_Planner::RRT_star of the created solver, passing the starting and ending states 
	of the problem that you want to solve. Finally, you can use I_Planner::Get_solution to get the obtained solution, i.e. the chain of states connecting the staring
	and the ending nodes, compliant with the constraints. In case a solution was not found, an empty path is returned.
	Another planning process can be invoked using the same object. In this case, the info about the last planning are detroyed and replaced with the new one.
	*/
	class Solver {
	public:
		virtual ~Solver() = default;

		Solver(const Solver&) = delete;
		Solver& operator=(const Solver&) = delete;

		Solver(ProblemPtr problemDescription);

		Solver(ProblemPtr problemDescription, std::unique_ptr<Strategy> solverStrategy);

		void setStrategy(std::unique_ptr<Strategy> solverStrategy);

		// useful to catch the solver to set custom options and then re-set it calling setStrategy
		std::unique_ptr<Strategy> extractStrategy();

		/** \brief Tries to solve the problem by executing the basic single tree RRT version (Section 1.2.1 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching tree.
		The data about the solutions of any previous problem solved are deleted.
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		void									setSteerTrials(const std::size_t& trials);

		// passing 0, the number of core is assumed
		void									setThreadAvailability(const std::size_t& threads = 0);

		std::size_t								getLastIterations() const;

		std::chrono::milliseconds				getLastElapsedTime() const;

		// copied
		std::vector<NodeState>					copyLastSolution() const;

		// moved
		std::vector<TreePtrConst>				extractLastTrees();

		std::size_t								getThreadAvailability() const;

		template<typename User>
		void									useProblem(const User& user) {
			std::lock_guard<std::mutex> lck(this->data->solverMutex);
			user(*this->data->problemsBattery.front().get());
		};

		void									saveTreesAfterSolve();
		void									discardTreesAfterSolve();

	private:
	// data
		std::shared_ptr<SolverData>						data;

		std::unique_ptr<SolutionInfo>					lastSolution = nullptr;

		std::unique_ptr<Strategy>						strategy = nullptr;
	};
}
#endif