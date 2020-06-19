/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include <omp.h>
#include "../Header/Planner_MT.h"
using namespace std;

namespace MT_RTT
{

	class Planner_shared_parall : public I_Planner_MT {
	public:
		Planner_shared_parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads) :
			I_Planner_MT( det_coeff, max_iter, handler, N_threads) {};
	protected:
		virtual void					  _RRT_basic(const Array& start, const Array& end);
		virtual void		_RRT_bidirectional(const Array& start, const Array& end);
		virtual void						 _RRT_star(const Array& start, const Array& end);
	private:
		template<typename Solver>
		void						_get_solution(vector<Solver>& Battery_solver) {

			bool life = true;

			int Number_threads = (int)Battery_solver.size();
			auto seeds = random_seeds(Number_threads);

#pragma omp parallel \
num_threads(Number_threads)
			{

				int th_id = omp_get_thread_num();
				srand(seeds[th_id]);

				Solver*  Solver_to_use = &Battery_solver[th_id];
				bool do_break = false;

				for (size_t k = 0; k < this->Iterations_Max; k += Number_threads) {
#pragma omp critical
					{
						if (!life)
							do_break = true;
					}
					if (do_break)
						break;

					Solver_to_use->Extend_within_iterations(1);
					if (Solver_to_use->Get_solution_was_found() && (!this->Cumulate_sol)) {
#pragma omp critical
						{
							life = false;
						}
						break;
					}
				}
			}

			__last_solution_info info;
			info.Iteration_done = Battery_solver[0].Get_Iterations();
			Solver::Get_best_solution(&info.Solution, Battery_solver);
			info.Trees = Battery_solver[0].Remove_Trees();
			this->Set_Solution(info);

		};


		class Tree_critical : public I_Tree_decorator {
		public:
			static vector<Tree_critical*>	   Init_Battery(const Array& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads);
		private:
			Tree_critical(Tree_concrete* concrete_to_wrap, const bool& destroy_wrap) :
				I_Tree_decorator(concrete_to_wrap, destroy_wrap) {};

			class Tree_concrete_critical : public Tree_concrete {
			public:
				Tree_concrete_critical(const Array& root_state, Node::I_Node_factory* handler, const size_t& N_threads) : Tree_concrete(root_state, handler, false) {
					this->__infoes.reserve(N_threads);
					this->__handlers.reserve(N_threads);
					for (size_t k = 0; k < N_threads; ++k) {
						this->__infoes.emplace_back();
						this->__infoes.back().target_reached = false;
						if (k == 0) this->__handlers.emplace_back(handler);
						else {
							auto clone = handler->copy();
							this->__handlers.emplace_back(clone.get());
							clone.release();
						}
					}
				};
				~Tree_concrete_critical() { for (size_t k = 1; k < this->__handlers.size(); ++k) delete this->__handlers[k]; };

				virtual Node::I_Node_factory* Get_Problem_Handler() { int th_id = omp_get_thread_num();  return this->__handlers[th_id]; };
			private:
				virtual	Node*				   Extend(const Node* target);
				virtual size_t				   __get_Nodes_size() {
					size_t S;
#pragma omp critical
					{
						S = this->Nodes.size();
					}
					return S;
				};
				virtual _extend_info* Get_extend_info() { int th_id = omp_get_thread_num();  return &this->__infoes[th_id]; };

				vector<_extend_info>		   __infoes;
				vector<Node::I_Node_factory*>  __handlers;
			};
		};

		class Tree_star_critical : public Tree_star  {
		public:
			static vector<Tree_star_critical*> Init_Battery(const Array& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads);
		private:
			Tree_star_critical(Tree_critical* to_wrap) : Tree_star(to_wrap, true) {};
			virtual	Node*					   Extend(const Node* target);
		};
	};
	unique_ptr<I_Planner>							I_Planner::Get_shared__parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads) { return unique_ptr<I_Planner>(new Planner_shared_parall(det_coeff, max_iter, handler, N_threads)); };



	Node* Planner_shared_parall::Tree_critical::Tree_concrete_critical::Extend(const Node* target) {

		Node* near_N = this->Nearest_Neighbour(target);
		int calling_thread = omp_get_thread_num();
		Node* steered = new Node(this->Get_Problem_Handler()->Steer(near_N, target, &this->__infoes[calling_thread].target_reached));
		if (steered->Get_State() == nullptr) {
			delete steered;
			return nullptr;
		}

		if (this->__infoes[calling_thread].target_reached && (!this->__infoes[calling_thread].random_or_deter)) {
			delete steered;
			return near_N;
		}

#pragma omp critical
		{
			this->Nodes.push_back(steered);
		}
		return steered;

	}

	Node* Planner_shared_parall::Tree_star_critical::Extend(const Node* target) {

		Node* added = this->I_Tree_decorator::Extend(target);
		if (added != nullptr) {
			if (this->Extend_reached_determ_target()) return added;

			std::list<Node2Node_Traj> rewird_to_do;
#pragma omp critical
			{
				this->Connect_to_best_Father_and_eval_Rewirds(&rewird_to_do, added);
				auto it_end = rewird_to_do.end();
				for (auto it = rewird_to_do.begin(); it != it_end; ++it)
					it->end->Set_Father(it->start, it->cost);
			}
		}
		return added;

	}

	vector<Planner_shared_parall::Tree_critical*>	 Planner_shared_parall::Tree_critical::Init_Battery(const Array& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads) {

		Tree_concrete_critical* T = new Tree_concrete_critical(root_state, problem_handler, N_threads);
		vector<Tree_critical*>  Battery;
		Battery.reserve(N_threads);
		Battery.emplace_back(new Tree_critical(T, true));
		for (size_t k = 1; k < N_threads; ++k) 
			Battery.emplace_back(new Tree_critical(T, false));
		return Battery;

	}

	vector<Planner_shared_parall::Tree_star_critical*>	 Planner_shared_parall::Tree_star_critical::Init_Battery(const Array& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads) {

		auto temp = Tree_critical::Init_Battery(root_state, problem_handler, N_threads);
		vector<Tree_star_critical*> Battery;
		Battery.reserve(N_threads);
		for (size_t k = 0; k < N_threads; ++k)
			Battery.emplace_back(new Tree_star_critical(temp[k]));
		return Battery;

	}



	void Planner_shared_parall::_RRT_basic(const Array& start, const Array& end) {

		auto T_battery = Tree_critical::Init_Battery(start, this->Handler, this->get_Threads());
		vector<Single_Extension_job> Battery_solver;
		this->Init_Single_Extension_battery(&Battery_solver , this->cast_to_I_Tree<Tree_critical>(T_battery), end);
		this->_get_solution(Battery_solver);

	}

	void Planner_shared_parall::_RRT_bidirectional(const Array& start, const Array& end) {

		auto T_battery_A = Tree_critical::Init_Battery(start, this->Handler, this->get_Threads());
		auto T_battery_B = Tree_critical::Init_Battery(end, this->Handler, this->get_Threads());
		vector<Bidirectional_Extension_job> Battery_solver;  
		this->Init_Bidirectional_Extension_battery(&Battery_solver, this->cast_to_I_Tree<Tree_critical>(T_battery_A), this->cast_to_I_Tree<Tree_critical>(T_battery_B));
		this->_get_solution(Battery_solver);

	}

	void Planner_shared_parall::_RRT_star(const Array& start, const Array& end) {

		auto T_battery = Tree_star_critical::Init_Battery(start, this->Handler, this->get_Threads());
		vector<Single_Extension_job> Battery_solver;
		this->Init_Single_Extension_battery(&Battery_solver, this->cast_to_I_Tree<Tree_star_critical>(T_battery), end);
		this->_get_solution(Battery_solver);

	}

}