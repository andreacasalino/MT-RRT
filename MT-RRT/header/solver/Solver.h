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
	/** @brief The kind of rrt strategy to use, refer to the ones described in Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation
     */
	enum RRTStrategy { Single, Bidir, Star };

	class Strategy;

	/** @brief Is produced internally to @Solver, every time a new planning problem 
	 * is solved. The various quantity can be then accessed using the getters of @Solver.
     */
	struct SolutionInfo {
		SolutionInfo(const NodeState& start, const NodeState& target);

		const NodeState start;
		const NodeState target;
		
		/** @brief elapsed time
		 */
		std::chrono::milliseconds		time = std::chrono::milliseconds(0);
		/** @brief iterations spent
		 */
		std::size_t						iterations = 0;
		/** @brief The sequence of states forming the solution to the planning problem.
		 * Is an empty vector in case a solution was not found
		 */
		std::vector<NodeState>			solution;
		/** @brief The trees extended and used in order to solve the problem
		 */
		std::vector<TreePtr>			trees;
	};

	struct SolverData {
		std::mutex solverMutex;
		/** @brief Each working thread should use one of the element in this battery
      	 */
		std::vector<ProblemPtr> problemsBattery;
		/** @brief The obtained tree(s) are saved after solving a planning problem, in case this parameter is set true.
		 * Otherwise they are deleted in order to save memory space
      	 */
		bool saveComputedTrees = false;
	};
	
	/** @brief Solver storing results of planning problem. Every time solve(...) is called, a new problem is solved and the results can be
	 * accessed using the getters provided in this interface. When another problem is solved calling again solve(...), the information
	 * regarding the previous problem are lost.
	 */
	class Solver {
	public:
		virtual ~Solver() = default;

		Solver(const Solver&) = delete;
		Solver& operator=(const Solver&) = delete;

		/** @param the problem description, i.e. the obect consumed by the solver to extend exploring trees and find solution(s)
		 *  @throw passing nullptr as problemDescription 
		 */
		Solver(ProblemPtr problemDescription);

		/** @param the problem description, i.e. the obect consumed by the solver to extend exploring trees and find solution(s)
		 *  @param the solving strategy to use for the following planning problems to solve (same as building the object with no
		 *  strategy and then call setStrategy(...))
		 *  @throw passing nullptr as problemDescription
		 */
		Solver(ProblemPtr problemDescription, std::unique_ptr<Strategy> solverStrategy);

		/** @param the set the solving strategy to use for the following planning problems
		 */
		void setStrategy(std::unique_ptr<Strategy> solverStrategy);

		/** @param remove the solving strategy stored in the solver. Useful to externally manipulate the strategy object
		 * (setting parameters for example) and then re-assing it using setStrategy(...)
		 */
		std::unique_ptr<Strategy> extractStrategy();

		/** @brief Tries to solve a new plannig problem, using the previously set strategy.
		 * Information regarding the solution(s) found are stored inside this object
		 * @param the staring state of the problem to solve
		 * @param the ending state of the problem to solve
		 * @param the rrt strategy to use
		 * @throw if no @Strategy was set before calling this method. 
		 * @throw if size of start is inconsistent
		 * @throw if size of end is inconsistent 
		 * @throw passing Bidir for  rrtStrategy for a problem that is not symmetric
		 */
		void solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		/** @param regulates the number of steering trials to use for following plans
		 */
		void									setSteerTrials(const std::size_t& trials);

		/** @param the threads to use for solving future problems, when using a multi-threading Strategy.
		 * pass 0 to assume the number of cores.
		 */
		void									setThreadAvailability(const std::size_t& threads = 0);

		/** @return the number of iterations required by the last planification
		 */
		std::size_t								getLastIterations() const;

		/** @return the time required by the last planification
		 */
		std::chrono::milliseconds				getLastElapsedTime() const;

		/** @return the starting configuration of the last solved problem
		 */
		NodeState								getLastStart() const;
		/** @return the target configuration of the last solved problem
		 */
		NodeState								getLastTarget() const;

		/** @return a copy of the solution, i.e. sequence of states, to the last solved planning problem.
		 */
		std::vector<NodeState>					copyLastSolution() const;

		/** @return extracts the trees obtained for solving the last problem. The trees are moved out and
		 * therefore a subsequent call to this method would return an empty vector.
		 * By default, the trees are NOT saved in order to save memory space. However you can specify to the solver
		 * to save the trees by calling saveTreesAfterSolve(). You can disable trees saving calling discardTreesAfterSolve()
		 */
		std::vector<TreePtrConst>				extractLastTrees();
		void									saveTreesAfterSolve();
		void									discardTreesAfterSolve();

		/** @return The number of threads that will be used when adopting a multi-threaded Strategy
		 */
		std::size_t								getThreadAvailability() const;

		/** @brief Use the problem stored inside this solver for some external purpose
		 * User should be a void operator accepting a const Problem& as input.
		 */
		template<typename User>
		void									useProblem(const User& user) const {
			std::lock_guard<std::mutex> lck(this->data->solverMutex);
			const Problem* prblPtr = this->data->problemsBattery.front().get();
			user(*prblPtr);
		};

	private:
	// data
		std::shared_ptr<SolverData>						data;

		std::unique_ptr<SolutionInfo>					lastSolution = nullptr;

		std::unique_ptr<Strategy>						strategy = nullptr;
	};
}
#endif