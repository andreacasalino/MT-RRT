/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include <omp.h>
#include <float.h>
#include <cmath>
#include "../Header/Planner_MT.h"
using namespace std;

namespace MT_RTT
{

	class Planner_query_parall : public I_Planner_MT {
	public:
		Planner_query_parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads) : I_Planner_MT(det_coeff, max_iter, handler, N_threads) {};
	protected:
		virtual void					  _RRT_basic(const Node_State& start, const Node_State& end);
		virtual void		      _RRT_bidirectional(const Node_State& start, const Node_State& end);
		virtual void					   _RRT_star(const Node_State& start, const Node_State& end);
	private:

		class I_Query {
		public:
			void operator()() {
				this->__job();
#pragma omp barrier
			};
		private:
			virtual void __job() = 0;
		};
		class Query_manager {
		public:
			Query_manager(const size_t& N_threads);

			//call it from slave
			void	Register_slave();									   

			//call it from master
			template<typename Q>
			void	Set_job_battery(vector<Q>& new_battery) {
				size_t K = this->Battery.size();
				for (size_t k = 0; k < K; k++)
					this->Battery[k] = &new_battery[k];
#pragma omp barrier
				this->Battery[0]->operator()();
			};
			void	Terminate();
		private:
			vector<I_Query*>	Battery;
		};


		class Tree_master : public Tree_concrete {
		public:
			Tree_master(const Node_State& root_state, Node::I_Node_factory* handler, const size_t& N_threads, Query_manager* man);
			~Tree_master();

			const vector<Node::I_Node_factory*>&  Get_Problem_handlers() { return this->Problem_handlers;};
			Query_manager*						  Get_manager() { return this->Query_man; };
		protected:
			virtual Node*						  Nearest_Neighbour(const Node* state);

			class Nearest_Neighbour_Query : public I_Query {
			public:
				Nearest_Neighbour_Query(Tree_concrete* tree, Node::I_Node_factory* problem) { this->Tree = tree; this->Problem = problem; };

				const Node*					target;
				Node*						nearest;
				float						nearest_cost;
			private:
				Tree_concrete*				Tree;
				Node::I_Node_factory*		Problem;

				virtual void __job();
			};
		// data
			vector<Node::I_Node_factory*>		Problem_handlers;
			vector<Nearest_Neighbour_Query>		Nearest_neighbour_query;
			Query_manager*						Query_man;
		};


		class Tree_master_star : public Tree_star {
		public:
			Tree_master_star(const Node_State& root_state, Node::I_Node_factory* handler, const size_t& N_threads, Query_manager* man);

		private:
			virtual void						  Near_set(std::list<Node*>* near_set, const Node* state);

			class Near_set_Query : public I_Query {
			public:
				Near_set_Query(Tree_star* tree, Node::I_Node_factory* problem) { this->Tree = tree; this->Problem = problem; };

				const Node*					target;
				list<Node*>					result;
			private:
				Tree_star*					Tree;
				Node::I_Node_factory*		Problem;

				virtual void __job();
			};
		// data
			vector<Near_set_Query>				Near_set_query;
		};


		template<typename Solver>
		void _get_solution(Solver* solver, Query_manager* manager) {

			int Number_threads = (int)this->get_Threads();
			auto seeds = random_seeds(Number_threads);

#pragma omp parallel \
num_threads(Number_threads)
			{

				int th_id = omp_get_thread_num();
				srand(seeds[th_id]);

				if (th_id == 0) {
					solver->Extend_within_iterations(this->Iterations_Max);
					manager->Terminate();
				}
				else  manager->Register_slave();
			}

			__last_solution_info info;
			info.Iteration_done = solver->Get_Iterations();
			solver->Get_best_solution(&info.Solution);
			info.Trees = solver->Remove_Trees();
			this->Set_Solution(info);
		};
	};
	unique_ptr<I_Planner> 	I_Planner::Get_query___parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads) { return unique_ptr<I_Planner>(new Planner_query_parall(det_coeff, max_iter, handler, N_threads)); };


	Planner_query_parall::Query_manager::Query_manager(const size_t& N_threads) {

		this->Battery.reserve(N_threads);
		for (size_t k = 0; k < N_threads; k++)
			this->Battery.push_back(nullptr);

	}

	void Planner_query_parall::Query_manager::Register_slave() {

		int th_id = omp_get_thread_num();
		while (true) {
#pragma omp barrier
			if (this->Battery[th_id] == nullptr) break;
			this->Battery[th_id]->operator()();
		}

	}

	void Planner_query_parall::Query_manager::Terminate() {
		size_t K = this->Battery.size();
		for (size_t k = 1; k < K; k++)
			this->Battery[k] = nullptr;
#pragma omp barrier
	}

	Planner_query_parall::Tree_master::Tree_master(const Node_State& root_state, Node::I_Node_factory* handler, const size_t& N_threads, Query_manager* man):
	Tree_concrete(root_state, handler, false){

		this->Query_man = man;
		this->Problem_handlers.reserve(N_threads);
		this->Nearest_neighbour_query.reserve(N_threads);
		
		this->Problem_handlers.emplace_back(handler);
		this->Nearest_neighbour_query.emplace_back(this, handler);
		for (size_t k = 1; k < N_threads; k++) {
			auto clone = handler->copy();
			Node::I_Node_factory* temp_pt = clone.get();
			clone.release();

			this->Problem_handlers.emplace_back(temp_pt);
			this->Nearest_neighbour_query.emplace_back(this, temp_pt);
		}

	}

	Planner_query_parall::Tree_master::~Tree_master() {
	
		size_t K = this->Problem_handlers.size();
		for (size_t k = 1; k < K; k++)
			delete this->Problem_handlers[k];

	}

	Node* Planner_query_parall::Tree_master::Nearest_Neighbour(const Node* state) {

		size_t K = this->Nearest_neighbour_query.size(), k;
		for (k = 0; k < K; k++)
			this->Nearest_neighbour_query[k].target = state;
		this->Query_man->Set_job_battery(this->Nearest_neighbour_query);

		for (k = 1; k < K; k++) {
			if (this->Nearest_neighbour_query[k].nearest_cost < this->Nearest_neighbour_query[0].nearest_cost) {
				this->Nearest_neighbour_query[0].nearest = this->Nearest_neighbour_query[k].nearest;
				this->Nearest_neighbour_query[0].nearest_cost = this->Nearest_neighbour_query[k].nearest_cost;
			}
		}
		return this->Nearest_neighbour_query[0].nearest;

	}

	void Planner_query_parall::Tree_master::Nearest_Neighbour_Query::__job() {

		this->nearest = nullptr;
		this->nearest_cost = FLT_MAX;
		auto Nodes = Get_Nodes_o(this->Tree);
		size_t Th_Id = omp_get_thread_num();
		if (Th_Id < Nodes->size()) {
			auto it_N = Nodes->begin();
			float att_cost;
			advance(it_N, Th_Id);
			this->Problem->Cost_to_go(&this->nearest_cost, *it_N, this->target);
			this->nearest = *it_N;
			size_t T = (size_t)omp_get_num_threads(), n = Th_Id + T;
			size_t NS = Nodes->size();
			while (n < NS) {
				advance(it_N, T);
				this->Problem->Cost_to_go(&att_cost, *it_N, this->target);
				if (att_cost < this->nearest_cost) {
					this->nearest_cost = att_cost;
					this->nearest = *it_N;
				}
				n += T;
			}
		}

	}

	Planner_query_parall::Tree_master_star::Tree_master_star(const Node_State& root_state, Node::I_Node_factory* handler, const size_t& N_threads, Query_manager*man):
	Tree_star(new Tree_master(root_state, handler, N_threads, man), true) {

		auto handlers = static_cast<Tree_master*>(this->Get_Wrapped())->Get_Problem_handlers();
		this->Near_set_query.reserve(N_threads);
		for (size_t k = 0; k < N_threads; k++)
			this->Near_set_query.emplace_back(this , handlers[k]);

	}

	void		Planner_query_parall::Tree_master_star::Near_set(std::list<Node*>* near_set, const Node* state) {

		size_t K = this->Near_set_query.size(), k;
		for (k = 0; k < K; k++)
			this->Near_set_query[k].target = state;
		static_cast<Tree_master*>(this->Get_Wrapped())->Get_manager()->Set_job_battery(this->Near_set_query);

		list<Node*>::iterator it, it_end;
		*near_set = move(this->Near_set_query[0].result);
		for (k = 1; k < K; k++) {
			it_end = this->Near_set_query[k].result.end();
			for (it = this->Near_set_query[k].result.begin(); it != it_end; it++)
				near_set->emplace_back(*it);
		}


	}

	void Planner_query_parall::Tree_master_star::Near_set_Query::__job() {

		this->result.clear();
		auto Nodes = Get_Nodes_o(this->Tree);
		size_t th_id = omp_get_thread_num();
		if (th_id < Nodes->size()) {
			float Tree_size = (float)Nodes->size();
			float ray = Problem->Get_Gamma() * powf(logf(Tree_size) / Tree_size, 1.0f / (float)Problem->Get_State_size());
			auto it_N = Nodes->begin();
			advance(it_N, th_id);
			Node* Nearest_neigh = this->target->Get_Father();
			if (th_id == 0)
				this->result.emplace_back(Nearest_neigh);
			float dist_att;
			size_t T = (size_t)omp_get_num_threads(), n = th_id + T;
			size_t NS = Nodes->size();
			while (n < NS) {
				advance(it_N, T);
				if (*it_N == this->target)
					break;
				if (*it_N != Nearest_neigh) {
					Problem->Cost_to_go(&dist_att, *it_N, this->target);
					if (dist_att <= ray)
						this->result.emplace_back(*it_N);
				}
				n += T;
			}
		}

	}


	void Planner_query_parall::_RRT_basic(const Node_State& start, const Node_State& end) {

		Query_manager man(this->get_Threads());
		auto Master = new Tree_master(start, this->Handler, this->get_Threads(), &man);
		Single_Extension_job Solver(Master, end, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&Solver , &man);

	}

	void Planner_query_parall::_RRT_bidirectional(const Node_State& start, const Node_State& end) {

		Query_manager man(this->get_Threads());
		auto Master_A = new Tree_master(start, this->Handler, this->get_Threads(), &man);
		auto Master_B = new Tree_master(end, this->Handler, this->get_Threads(), &man);
		Bidirectional_Extension_job Solver(Master_A, Master_B, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&Solver, &man);

	}

	void Planner_query_parall::_RRT_star(const Node_State& start, const Node_State& end) {

		Query_manager man(this->get_Threads());
		auto Master = new Tree_master_star(start, this->Handler, this->get_Threads(), &man);
		Single_Extension_job Solver(Master, end, &this->Deterministic_coefficient, &this->Cumulate_sol);
		this->_get_solution(&Solver, &man);

	}

}