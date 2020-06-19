/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __MT_RRT_PLANNER_H__
#define __MT_RRT_PLANNER_H__

#include "Tree.h"

namespace MT_RTT
{

	/** \brief Interface for a planner.
	\details For solving a planning problem you must first build the solver, using 
	I_Planner::Get_canonical, I_Planner::Get_query___parall, I_Planner::Get_shared__parall, I_Planner::Get_copied__parall or I_Planner::Get_multi_ag_parall. 
	Then you can call I_Planner::RRT_basic, I_Planner::RRT_bidirectional or I_Planner::RRT_star of the created solver, passing the starting and ending states 
	of the problem that you want to solve. Finally, you can use I_Planner::Get_solution to get the obtained solution, i.e. the chain of states connecting the staring
	and the ending nodes, compliant with the constraints. In case a solution was not found, an empty path is returned.
	Another planning process can be invoked using the same object. In this case, the info about the last planning are detroyed and replaced with the new one.
	*/
	class I_Planner {
	public:
		virtual ~I_Planner();

		I_Planner(const I_Planner&) = delete;
		void operator=(const I_Planner&) = delete;

		/** \brief Use this method to enable the cumulation of the solution also for non RRT* versions of the planner.
		\details In case the cumulation is enabled, the search is not arrested when a first solution is found, but is 
		kept active till the maximum number of iterations is reached. All the solutions found are stored and the best one
		is selected at the end. This solution will be the one externally accesible.
		*/
		void									Cumulate_solutions() { this->Cumulate_sol = true; };

		/** \brief Tries to solve the problem by executing the basic single tree RRT version (Section 1.2.1 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching tree. 
		The data about the solutions of any previous problem solved are deleted.
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void									RRT_basic(const Array& start, const Array& end);
		
		/** \brief Tries to solve the problem by executing the bidirectional RRT version (Section 1.2.2 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching trees. 
		The data about the solutions of any previous problem solved are deleted.
		This planning strategy cannot be adopted for non symmetric problem, see also Node::I_Node_factory::Get_symm_flag
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/
		void									RRT_bidirectional(const Array& start, const Array& end);
		
		/** \brief Tries to solve the problem by executing the RRT* version (Section 1.2.3 and the Sections contained in Chapter 3 of the documentation) of the solver represented by this object,
		step C of the pipeline presented in Section 1.3 of the documentation.
		\details The solution found is internally stored, as well as the computed searching trees. 
		The data about the solutions of any previous problem solved are deleted. When invoking this function 
		the cumulation of the solutions is aitomatically enabled.
		* @param[in] start the staring state of the problem to solve
		* @param[in] end   the ending state of the problem to solve
		*/		
		void									RRT_star(const Array& start, const Array& end);



	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//The methods below return a meaningful result only when invoked after a call to RRT_basic, RRT_bidirectional or RRT_star
		
		/** \brief Access the number of iterations performed by the solver for trying to solve the last specified planning problem.
		\details Result is 0 in case no problems were solved at the time of invoking this function.
		* @param[out] return the number of iterations done
		*/	
		size_t									Get_Iteration_done();
		
		/** \brief Access the number of iterations performed by the solver for trying to solve the last specified planning problem,
		step D.1 of the pipeline presented in Section 1.3 of the documentation.
		\details An empty list is returned in case the solution was not found.
		* @param[out] return the number of iterations done
		*/	
		std::list<Array>					    Get_solution();
		
		/** \brief Returns a json structure describing the searching trees computed when solving the last specified planning problem,
		step D.2 of the pipeline presented in Section 1.3 of the documentation.
		\details An empty structure is returned in case no problems were solved at the time of invoking this function.
		* @param[out] return a json structure describing the searching tree computed to solve the last problem (move is internally called when returning the result).
		*/	
		std::string								Get_Trees_as_JSON();
		
		/** \brief Append to the passed string a json structure describing the sequence of states representing the last solution found,
		step D.2 of the pipeline presented in Section 1.3 of the documentation.
		\details An empty structure is returned in case no problems were solved at the time of invoking this function.
		* @param[out] return a json structure with the waypoints representing the solution (move is internally called when returning the result).
		*/	
		std::string								Get_Solution_as_JSON();

	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//The methods allows to create a particular solver, for use it at a second stage to solve one or several planning problems.
	//You need to build a solver for addressing step B of the pipeline presented in Section 1.3 of the documentation.
	
		/** \brief Get a canonical solver implementing the standard non parallel versions of the RRT algorith (Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation).
		\details The creation of this kind of solver addresses step B of the pipeline presented in Section 1.3 of the documentation.
		* @param[out] return the solver to use later
		* @param[in] det_coeff the percentage of times for which a deterministic extension must be tried to get a solution (the parameter \sigma of the algorithms 
		exposed in Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation)
		* @param[in] max_iter the maximum number of iterations to consider when soving the planning problems
		* @param[in] handler the object describing the planning problem to solve
		*/	
		static std::unique_ptr<I_Planner>							Get_canonical(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler);

		/** \brief Get the solver for which the query operations are parallelized (Section 3.0.1 of the documentation).
		\details The creation of this kind of solver addresses step B of the pipeline presented in Section 1.3 of the documentation.
		* @param[out] return the solver to use later
		* @param[in] det_coeff same meaning as in I_Planner::Get_canonical
		* @param[in] max_iter same meaning as in I_Planner::Get_canonical
		* @param[in] handler same meaning as in I_Planner::Get_canonical
		* @param[in] N_threads the number of threads to use. When passing 0, the maximal number admitted by this machine is used.
		*/	
		static std::unique_ptr<I_Planner>							Get_query___parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads = 0);

		/** \brief Get the solver for which many threads synchronize to expand a common tree (Section 3.0.2 of the documentation).
		\details The creation of this kind of solver addresses step B of the pipeline presented in Section 1.3 of the documentation.
		* @param[out] return the solver to use later
		* @param[in] det_coeff same meaning as in I_Planner::Get_canonical
		* @param[in] max_iter same meaning as in I_Planner::Get_canonical
		* @param[in] handler same meaning as in I_Planner::Get_canonical
		* @param[in] N_threads same meaning as in I_Planner::Get_query___parall
		*/	
		static std::unique_ptr<I_Planner>							Get_shared__parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads = 0);
		
		/** \brief Get the solver for which many threads expands its own copy of the searching tree (Section 3.0.3 of the documentation).
		\details The creation of this kind of solver addresses step B of the pipeline presented in Section 1.3 of the documentation.
		* @param[out] return the solver to use later
		* @param[in] det_coeff same meaning as in I_Planner::Get_canonical
		* @param[in] max_iter same meaning as in I_Planner::Get_canonical
		* @param[in] handler same meaning as in I_Planner::Get_canonical
		* @param[in] N_threads same meaning as in I_Planner::Get_query___parall
		* @param[in] reallignement_percentage the size of the explorative batches (percentage w.r.t. max_iter), see Section 3.0.3 of the documentation. 
		*/	
		static std::unique_ptr<I_Planner>							Get_copied__parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads = 0, const float& reallignement_percentage = 0.1f);
				
		/** \brief Get the solver implementing the multi agents version of the RRT algorithm (Section 3.0.4 of the documentation).
		\details The creation of this kind of solver addresses step B of the pipeline presented in Section 1.3 of the documentation.
		* @param[out] return the solver to use later
		* @param[in] det_coeff same meaning as in I_Planner::Get_canonical
		* @param[in] max_iter same meaning as in I_Planner::Get_canonical
		* @param[in] handler same meaning as in I_Planner::Get_canonical
		* @param[in] N_threads same meaning as in I_Planner::Get_query___parall
		* @param[in] reallignement_percentage similar meaning as in I_Planner::Get_copied__parall, see Section 3.0.4 of the documentation.
		*/	
		static std::unique_ptr<I_Planner>							Get_multi_ag_parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads = 0, const float& reallignement_percentage = 0.1f);
		
	//
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	protected:
		/** \brief Constructor
		* @param[in] det_coeff same meaning as in I_Planner::Get_canonical
		* @param[in] max_iter same meaning as in I_Planner::Get_canonical
		* @param[in] handler same meaning as in I_Planner::Get_canonical
		*/	
		I_Planner(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler);

		/** \brief the method overrided by all the derived planner for performing an RRT search
		*/			
		virtual void					  _RRT_basic(const Array& start, const Array& end) = 0;
		/** \brief the method overrided by all the derived planner for performing a bidirectionl search
		*/			
		virtual void					  _RRT_bidirectional(const Array& start, const Array& end) = 0;
		/** \brief the method overrided by all the derived planner for performing an RRT* search
		*/			
		virtual void					  _RRT_star(const Array& start, const Array& end) = 0;

		struct __last_solution_info {
			size_t							Iteration_done;
			std::list<Array>				Solution;
			std::list<I_Tree*>				Trees;
		};
		void								Set_Solution(const __last_solution_info& last_sol);

	// data
		Node::I_Node_factory*							Handler;
		float											Deterministic_coefficient;
		size_t											Iterations_Max;
		bool											Cumulate_sol;
	private:
		void							__clean_trees();

	// data
		__last_solution_info*								Last_solution;
	};

}
#endif