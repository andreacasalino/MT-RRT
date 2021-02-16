/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __MT_RRT_EXT_STRTGY_H__
#define __MT_RRT_EXT_STRTGY_H__

#include "Tree.h"
#include <vector>

//#define _DISPLAY_ITERATIONS //when defined, the number of iterations reached by the solver is printed on the console. Used mainly for debug

namespace MT_RTT
{

	/** \brief Interface for handling the extension steps involved in each RRT strategy. 
	\details Solutions eventually found while extending the trees, are saved and stored inside this kind of objects
	*/
	template<typename Sol_found>
	class I_Extension_job {
	public:
		virtual ~I_Extension_job() {};

		/** \brief Perform the specified number of estensions on a wrapped tree(s).
		\details This function may be called multiple times, for performing batch of extensions.
		If the cumulation of the solution was not enabled, calling this method when a solution was already
		found raise an exception.
		* @param[in] Iterations the number of extensions to perform
		*/
		virtual void					Extend_within_iterations(const size_t& Iterations) = 0;

		/** \brief Returns true in case at least a solution was found in the iterations so far
		done by this extender, false otherwise.
		*/
		const bool& Get_solution_was_found() { return this->A_solution_was_found; };

		/** \brief Get the extensions so far done.
		*/
		const size_t& Get_Iterations() { return this->Iterations_done; };

		/** \brief Move outside of this object all the trees it contains. 
		\details If the trees are not removed, they are all destroyed when destroying this object
		* @param[out] return the trees that are removed from this object
		*/
		virtual	std::list<I_Tree*>		Remove_Trees() = 0;

		/** \brief Returns the best cost solution found so far.
		* @param[out] solution the list of states representing the solution. Values are copied from the correspoding node states.
		*/
		void							Get_best_solution(std::list<Array>* solution) { this->__Get_best_solution(solution , this->Solutions_found); };

		/** \brief Similar to Get_best_solution(std::list<Array>* solution), but taking the best solution among the ones contained
		in the passed array of extenders.
		\details Ext should be something deriving from I_Extension_job<Sol_found>.
		* @param[in] battery the array of extender to consider
		* @param[out] solution the list of states representing the solution. Values are copied from the correspoding node states.
		*/
		template<typename Ext>
		static void						Get_best_solution(std::list<Array>* solution, std::vector<Ext>& battery) {

			std::list<Sol_found> solutions;
			size_t K = battery.size();
			for (size_t k = 0; k < K; ++k) {
				I_Extension_job<Sol_found>* pt = &battery[k];
				auto it_end = pt->Solutions_found.end();
				for (auto it = pt->Solutions_found.begin(); it != it_end; ++it)
					solutions.emplace_back(*it);
			}
			I_Extension_job<Sol_found>* pt = &battery[0];
			pt->__Get_best_solution(solution, solutions);

		};
	protected:
		/** \brief Constructor
		* @param[in] det_coeff a pointer to the value regulating the probability of a deterministic extension (refer to the parameter \sigma of the algorithms exposed in Sections 1.2.1, 1.2.2 and 1.2.3 
		of the documentation)
		* @param[in] cumul_sol a pointer to the boolean that explaining whether to accumulate or not feasible found solutions
		*/
		I_Extension_job(const float* det_coeff, const bool* cumul_sol) :
			A_solution_was_found(false), Cumulate_sol(cumul_sol), Deterministic_coefficient(det_coeff) {
			this->Iterations_done = 0;
		};

		void							Check_Extension() { if ((this->A_solution_was_found) && (!this->Cumulate_sol)) throw 0; }
		virtual void					__Get_best_solution(std::list<Array>* solution, const std::list<Sol_found>& solutions) = 0;
		static const Sol_found*				get_best_solution(const std::list<Sol_found>& solutions) {

			auto it = solutions.begin();
			auto it_end = solutions.end();
			const Sol_found* best = &(*it);
			++it;
			for (it = it; it != it_end; ++it) {
				if (*it < *best)
					best = &(*it);
			}
			return best;

		}
	// data
		bool							 A_solution_was_found;
		const bool*						 Cumulate_sol; 
		const float*					 Deterministic_coefficient;
		size_t							 Iterations_done;
		std::list<Sol_found>			 Solutions_found;
	};



	struct single_solution {
		single_solution(const float& c, const Node* p) : cost(c), peer(p) {};
		single_solution(const single_solution& o) : cost(o.cost), peer(o.peer) {};
		bool operator ==(const single_solution& o) const { return (this->peer == o.peer); };
		bool operator  <(const single_solution& o) const { return (this->cost < o.cost); };

		float		cost;
		const Node* peer;
	};
	/** \brief Handles the single tree extension strategy.
	\details A tree is extended within the given iterations, both randomly and toward a specified target state. 
	The deterministic extension finds the nearest neighbour of the tree to the target and try to steer it till 
    reaching it. 
	Here the solution consists in the pointer to the node that reached the target.
	*/
	class Single_Extension_job : public I_Extension_job<single_solution> {
	public:
		/** \brief Constructor
		* @param[in] to_extend the tree to absorb and extend
		* @param[in] target the target node that must be connected to the tree to extend
		* @param[in] det_coeff same as in I_Extension_job::I_Extension_job
		* @param[in] cumul_sol same as in I_Extension_job::I_Extension_job 
		* @param[in] del_trg when passed true, the absorbed target is deleted in the constructor of this object
		*/
		Single_Extension_job(I_Tree* to_extend, const Array& target, const float* det_coeff, const bool* cumul_sol) :
			I_Extension_job<single_solution>(det_coeff, cumul_sol), T(to_extend), Target(new Node(to_extend->Get_Problem_Handler()->New_root(target)))  { };

		~Single_Extension_job() { delete this->T; delete this->Target; };

		virtual void					Extend_within_iterations(const size_t& Iterations);

		virtual	std::list<I_Tree*>		Remove_Trees();
	protected:
		virtual void					__Get_best_solution(std::list<Array>* solution, const std::list<single_solution>& solutions);
	// data
		I_Tree*							T;
		Node*							Target;
	};



	struct bidir_solution {
		bidir_solution(const float& c, const Node* A, const Node* B) : cost(c), peer_A(A), peer_B(B) {};
		bidir_solution(const bidir_solution& o) : cost(o.cost), peer_A(o.peer_A), peer_B(o.peer_B) {};
		bool operator ==(const bidir_solution& o) const { return  ((o.peer_A == this->peer_A) && (o.peer_B == this->peer_B)); };
		bool operator  <(const bidir_solution& o) const { return (this->cost < o.cost); };

		float		cost;
		const Node* peer_A;
		const Node* peer_B;
	};
	/** \brief Handles the bidirectional extension strategy (Section 1.2.2 of the documentation).
	\details Two trees, A and B, are simultaneously extended within the given iterations, trying to establish a connection between them. 
	A deterministic extension is performed sometimes, which tries to connect the nearest neighbour in tree A, to the root of tree B (or vice-versa).
	Here the solution consists in the pair of pointers to the nodes in tree A and B that touches themself, establishing a connection between the two trees.
	*/
	class Bidirectional_Extension_job : public I_Extension_job<bidir_solution> {
	public:
		/** \brief Constructor
		* @param[in] to_extend_A the tree A to absorb and extend toward B
		* @param[in] to_extend_B the tree B to absorb and extend toward A
		* @param[in] det_coeff same as in I_Extension_job::I_Extension_job
		* @param[in] cumul_sol same as in I_Extension_job::I_Extension_job 
		*/
		Bidirectional_Extension_job(I_Tree* to_extend_A, I_Tree* to_extend_B, const float* det_coeff, const bool* cumul_sol) : I_Extension_job<bidir_solution>(det_coeff, cumul_sol) {

			this->T_a = to_extend_A;
			this->T_b = to_extend_B;

		};

		~Bidirectional_Extension_job() { delete this->T_a;  delete this->T_b; };

		virtual void					Extend_within_iterations(const size_t& Iterations);

		virtual	std::list<I_Tree*>		Remove_Trees();
	protected:
		void							compute_sol(bidir_solution& sol, const Node* N1, const Node* N2, const bool& caso);
		void							compute_cost(bidir_solution& sol);
		virtual void					__Get_best_solution(std::list<Array>* solution, const std::list<bidir_solution>& solutions);
	// data
		I_Tree*								T_a;
		I_Tree*								T_b;
	};

}

#endif