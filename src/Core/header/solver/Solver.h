/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SOLVER_H
#define MT_RRT_SOLVER_H

#include <problem/Problem.h>
#include <solver/Tree.h>
#include <mutex>

namespace mt::solver {
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

		Solver(problem::ProblemPtr problemDescription);

		enum Strategy { Serial, MtQueryParall, MtSharedTree, MtCopiedTrees, MtMultiAgent };

		enum RRTStrategy { Single, Bidir, Star };

		/** \brief Tries to solve the problem by executing the basic single tree RRT version (Section 1.2.1 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching tree.
		The data about the solutions of any previous problem solved are deleted.
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void RRTSingle(const NodeState& start, const NodeState& end, const Strategy& strategy = Serial);

		/** \brief Tries to solve the problem by executing the bidirectional RRT version (Section 1.2.2 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching trees.
		The data about the solutions of any previous problem solved are deleted.
		This planning strategy cannot be adopted for non symmetric problem, see also Node::I_Node_factory::Get_symm_flag
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void RRTConnect(const NodeState& start, const NodeState& end, const Strategy& strategy = Serial);

		/** \brief Tries to solve the problem by executing the RRT* version (Section 1.2.3 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching trees.
		The data about the solutions of any previous problem solved are deleted. When invoking this function
		the cumulation of the solutions is aitomatically enabled.
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void RRTStar(const NodeState& start, const NodeState& end, const Strategy& strategy = Serial);



		void									setThreadAvailability(const std::size_t& threads);

		void									setDeterminism(const double& coeff);

		void									setMaxIterations(const std::size_t& iter);

		/** \brief Use this method to enable the cumulation of the solution also for non RRT* versions of the planner.
		\details In case the cumulation is enabled, the search is not arrested when a first solution is found, but is 
		kept active till the maximum number of iterations is reached. All the solutions found are stored and the best one
		is selected at the end. This solution will be the one externally accesible.
		*/
		void									setCumulateOption(const bool& val = true);



		std::size_t								getLastIterations() const;

		// copied
		std::vector<NodeState>					getLastSolution() const;

		// moved
		std::vector<tree::TreePtr>					getLastTrees();

	private:
		struct SolutionInfo {
			std::size_t						iterations;
			std::vector<NodeState>			solution;
			std::vector<tree::TreePtr>		trees;
		};

		struct Parameters {
			double											Deterministic_coefficient = 0.2f;
			size_t											Iterations_Max = 1000;
			bool											Cumulate_sol = false;
		};

		std::unique_ptr<SolutionInfo> serialStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		std::unique_ptr<SolutionInfo> queryParallStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		std::unique_ptr<SolutionInfo> sharedTreeStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		std::unique_ptr<SolutionInfo> copiedTreesStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

		std::unique_ptr<SolutionInfo> multiAgentStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy);

	// data
		mutable std::mutex								dataMtx;
		std::vector<problem::ProblemPtr>				problemcopies;
		Parameters										parameters;

		std::unique_ptr<SolutionInfo>					lastSolution = nullptr;
	};
}
#endif