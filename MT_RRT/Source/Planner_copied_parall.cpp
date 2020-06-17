/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Planner.h"
#include <omp.h>
#include <cmath>
#include "../Header/Planner_MT.h"
using namespace std;

namespace MT_RTT
{

	class Planner_copied_parall : public I_Planner_MT {
	public:
		Planner_copied_parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads, const float& reallignement_percentage) : 
			I_Planner_MT(det_coeff, max_iter, handler, N_threads), Reallignement_prctg(reallignement_percentage) {};
	protected:
		virtual void					  _RRT_basic(const Node_State& start, const Node_State& end);
		virtual void		_RRT_bidirectional(const Node_State& start, const Node_State& end);
		virtual void						 _RRT_star(const Node_State& start, const Node_State& end);
	private:

		template<typename Tree>
		class Absorber {
		public:
			Absorber() {};

			void Absorb_from_others(const int& th_id) {

				auto it_end = this->batteries.end();
				for (auto it = this->batteries.begin(); it != it_end; it++) 
					(*it)[th_id]->Absorb_from_others();

			}
			void Add_battery(const vector<Tree*>& battery) { this->batteries.emplace_back(battery); }
		private:
			list<vector<Tree*>>  batteries;
		};

		template<typename Solver, typename Tree>
		void						_get_solution(vector<Solver>& Battery_solver, Absorber<Tree>& Processor) {

			bool life = true;
			int Threads = (int)this->get_Threads();
			size_t Batch_size = (size_t)ceil(this->Reallignement_prctg * (float) this->Iterations_Max / (float)(Threads) );
			auto seeds = random_seeds(Threads);

#pragma omp parallel \
num_threads(Threads)
			{
				int th_id = omp_get_thread_num();
				srand(seeds[th_id]);

				Solver* Solver_to_use = &Battery_solver[th_id];
				for (size_t k = 0; k < this->Iterations_Max; k +=  Batch_size * Threads) {
					if (!life) break;

					Solver_to_use->Extend_within_iterations(Batch_size);
#pragma omp barrier
					Processor.Absorb_from_others(th_id);
					if (Solver_to_use->Get_solution_was_found() && (!this->Cumulate_sol)) {
#pragma omp critical
						{
							life = false;
						}
					}
#pragma omp barrier
				}
				Processor.Absorb_from_others(th_id);
			}

			__last_solution_info info;
			info.Iteration_done = Battery_solver[0].Get_Iterations();
			Solver::Get_best_solution(&info.Solution, Battery_solver);
			info.Trees = Battery_solver[0].Remove_Trees();
			this->Set_Solution(info);

		};

		template<typename T>
		class linked_buffers {
		public:
			linked_buffers() {};

			template<typename E> //E should be something deriving from linked_buffers<T>
			static void bind_buffers(std::vector<E*>& buffers) {

				size_t K = buffers.size(), k, k2;
			//create Incoming buffers
				for (k = 0; k < K; k++) {
					buffers[k]->Incomings.reserve(K-1);
					buffers[k]->Outgoings.reserve(K-1);
					for (k2 = 1; k2 < K; k2++) 
						buffers[k]->Incomings.emplace_back();
				}
			//link incomings to outgoings
				vector<size_t> inc_used;
				inc_used.reserve(K);
				for (k = 0; k < K; k++)
					inc_used.emplace_back(0);
				for (k = 0; k < K; k++) {
					for (k2 = 0; k2 < K; k2++) {
						if (k2 != k) {
							buffers[k]->Outgoings.emplace_back(&buffers[k2]->Incomings[inc_used[k2]]);
							inc_used[k2]++;
						}
					}
				}

			};

		protected:
			// data
			std::vector<std::list<T>>	Incomings;
			std::vector<std::list<T>*>	Outgoings;
		};

		class Tree_linked: public Tree_concrete, public linked_buffers<Node*> {
		public:
			class Node_linked : public Node {
			public:
				static void	create_linked_roots(const Node_State& root_state, const vector<Tree_linked*>& trees);
				static void	dispatch_last_added(Tree_linked* tree);

				const std::vector<Node_linked*>* Get_Copies() const { return &this->Copies; };
			private:
				Node_linked(Node& o) : Node(move(o)) {};

				static void							__link(vector<Node_linked*>& battery);
			// data
				std::vector<Node_linked*>			Copies;
			};

			static vector<Tree_linked*>				Init_Battery(const Node_State& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads);
			void									Absorb_from_others();
		private:
			Tree_linked(Node::I_Node_factory* handler, const bool& clone_handler, const size_t& th_id) : Tree_concrete(handler, clone_handler), Th_id(th_id) {  };
			virtual	Node*							Extend(const Node* target);

			size_t			Th_id;
		};

		class Tree_star_linked : public Tree_star, public linked_buffers<Tree_star::Node2Node_Traj> {
		public:
			static vector<Tree_star_linked*>		Init_Battery(const Node_State& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads);
			void									Absorb_from_others();

			class Extension_star_linked : public Single_Extension_job {
			public:
				Extension_star_linked(I_Tree* to_extend, const Node_State& target, const float* det_coeff, const bool* cumul_sol) :
					Single_Extension_job(to_extend, target, det_coeff, cumul_sol) {};
			private:
				virtual void						__Get_best_solution(std::list<Node_State>* solution, const std::list<single_solution>& solutions);
				const Node*							Get_best_Father(const Node* attual);
			};
		private:
			Tree_star_linked(Tree_linked* to_wrap) : Tree_star( to_wrap, true) {  };
			virtual	Node*							Extend(const Node* target);
		};

		float  Reallignement_prctg;
	};
	unique_ptr<I_Planner>							I_Planner::Get_copied__parall(const float& det_coeff, const size_t& max_iter, Node::I_Node_factory* handler, const size_t& N_threads, const float& reallignement_percentage) { return unique_ptr<I_Planner>(new Planner_copied_parall(det_coeff, max_iter, handler, N_threads, reallignement_percentage)); };


	void Planner_copied_parall::Tree_linked::Absorb_from_others() {

		auto Nodes = this->Get_Nodes();
		auto it2 = this->Incomings.begin()->begin();
		auto it2_end = it2;
		auto it_end = this->Incomings.end();
		for (auto it = this->Incomings.begin(); it != it_end; it++) {
			it2_end = it->end();
			for (it2 = it->begin(); it2 != it2_end; it2++)
				Nodes->emplace_back(*it2);
			it->clear();
		}

	}

	void Planner_copied_parall::Tree_linked::Node_linked::create_linked_roots(const Node_State& root_state, const vector<Tree_linked*>& trees) {

		vector<Node_linked*> battery;
		size_t K = trees.size();
		battery.reserve(K);
		auto problem = trees.front()->Get_Problem_Handler();
		for (size_t k = 0; k < K; k++) {
			Node root = problem->New_root(root_state);
			Node_linked* temp = new Node_linked(root);
			battery.emplace_back(temp);
			trees[k]->Get_Nodes()->emplace_back(temp);
		}
		__link(battery);

	}

	void Planner_copied_parall::Tree_linked::Node_linked::dispatch_last_added(Tree_linked* generating_tree) {

		auto Nodes = generating_tree->Get_Nodes();
		Node* node = Nodes->back();
		auto fathers = static_cast<Node_linked*>(node->Get_Father())->Get_Copies();
		vector<Node_linked*> battery;
		size_t N = generating_tree->Outgoings.size() + 1;
		battery.reserve(N);
		size_t kf = 0;
		const float& cost = node->Get_Cost_from_father();
		auto problem = generating_tree->Get_Problem_Handler();
		for (size_t k = 0; k < N; k++) {
			if (k == generating_tree->Th_id)  
				battery.emplace_back(nullptr);
			else {
				Node temp = problem->Clone_Node(*node);
				temp.Set_Father((*fathers)[kf], cost);
				battery.emplace_back(new Node_linked(temp));
				generating_tree->Outgoings[kf]->emplace_back(battery.back());
				kf++;
			}
		}
		battery[generating_tree->Th_id] = new Node_linked(*node);
		delete node;
		Nodes->back() = battery[generating_tree->Th_id];
		__link(battery);

	}

	void Planner_copied_parall::Tree_linked::Node_linked::__link(vector<Node_linked*>& battery) {

		size_t k, K = battery.size(), k2;
		for (k = 0; k < K; k++) {
			battery[k]->Copies.reserve(K - 1);
			for (k2 = 0; k2 < K; k2++) {
				if (k != k2) battery[k]->Copies.emplace_back(battery[k2]);
			}
		}

	}

	Node* Planner_copied_parall::Tree_linked::Extend(const Node* target) {

		Node* added_node = this->Tree_concrete::Extend(target);
		if (added_node != nullptr) {
			if (this->Extend_reached_determ_target()) return added_node;

			Node_linked::dispatch_last_added(this);
			added_node = this->Get_Nodes()->back();
		}
		return added_node;

	}

	void Planner_copied_parall::Tree_star_linked::Absorb_from_others() {

		static_cast<Tree_linked*>(this->Get_Wrapped())->Absorb_from_others();
		//round_robin_rewird_gather(this->Incomings);

	}

	Node*	Planner_copied_parall::Tree_star_linked::Extend(const Node* target) {

		Node* steered = this->I_Tree_decorator::Extend(target);
		if (steered != nullptr) {
			if (this->Extend_reached_determ_target()) return steered;

			Tree_linked::Node_linked* added = static_cast<Tree_linked::Node_linked*>(steered);
			Node* old_father = added->Get_Father();
			std::list<Node2Node_Traj> rewird_to_do;
			this->Connect_to_best_Father_and_eval_Rewirds(&rewird_to_do, added);
			const vector<Tree_linked::Node_linked*>* fathers = nullptr;
			const vector<Tree_linked::Node_linked*>* copies = nullptr;
			size_t k, K = this->Outgoings.size();

			Node* new_father = added->Get_Father();
			if (new_father != old_father) {
				fathers = static_cast<Tree_linked::Node_linked*>(new_father)->Get_Copies();
				copies = added->Get_Copies();
				const float& c = added->Get_Cost_from_father();
				for (k = 0; k < K; k++) 
					(*copies)[k]->Set_Father((*fathers)[k], c); //rewird can be done here because these nodes are not already inserted in the others thread tree
			}

			auto it_end = rewird_to_do.end();
			for (auto it = rewird_to_do.begin(); it != it_end; it++) {
				//fathers = static_cast<Tree_linked::Node_linked*>(it->start)->Get_Copies();
				//copies = static_cast<Tree_linked::Node_linked*>(it->end)->Get_Copies();
				//for (k = 0; k < K; k++) {
				//	this->Outgoings[k]->emplace_back();
				//	this->Outgoings[k]->back().cost = it->cost;
				//	this->Outgoings[k]->back().start = (*fathers)[k];
				//	this->Outgoings[k]->back().end = (*copies)[k];
				//}
				it->end->Set_Father(it->start, it->cost);
			}

		}
		return steered;

	}

	void	Planner_copied_parall::Tree_star_linked::Extension_star_linked::__Get_best_solution(std::list<Node_State>* solution, const std::list<single_solution>& solutions) {

		solution->clear();
		if (solutions.empty()) return;

		auto best_solution = get_best_solution(solutions);
		auto problem = this->T->Get_Problem_Handler();
		size_t S = problem->Get_State_size();

		solution->emplace_front(this->Target->Get_State(), S);
		const Node* cursor = best_solution->peer;
		while (cursor != nullptr) {
			solution->emplace_front(cursor->Get_State(), S);
			cursor = this->Get_best_Father(cursor);
		}

	};

	const Node* Planner_copied_parall::Tree_star_linked::Extension_star_linked::Get_best_Father(const Node* attual) {

		const Node* best_father = attual->Get_Father();
		if (best_father == nullptr) return nullptr;
		const std::vector<Tree_linked::Node_linked*>* attual_clone = static_cast<const Tree_linked::Node_linked*>(attual)->Get_Copies();
		
		float best_cost, att_cost;
		attual->Cost_to_root(&best_cost);
		size_t K = attual_clone->size();
		for (size_t k = 0; k < K; k++) {
			(*attual_clone)[k]->Cost_to_root(&att_cost);
			if (att_cost < best_cost) {
				best_cost = att_cost;
				best_father = (*attual_clone)[k]->Get_Father();
			}
		}
		return best_father;

	}

	vector<Planner_copied_parall::Tree_linked*>	Planner_copied_parall::Tree_linked::Init_Battery(const Node_State& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads) {

		vector<Planner_copied_parall::Tree_linked*> Battery;
		Battery.reserve(N_threads);
		Battery.emplace_back(new Tree_linked(problem_handler, false, 0));
		for (size_t k = 1; k < N_threads; k++)
			Battery.emplace_back(new Tree_linked(problem_handler, true, k));
		linked_buffers<Node*>::bind_buffers(Battery);
		Node_linked::create_linked_roots(root_state, Battery);
		return Battery;

	}

	vector<Planner_copied_parall::Tree_star_linked*> Planner_copied_parall::Tree_star_linked::Init_Battery(const Node_State& root_state, Node::I_Node_factory* problem_handler, const size_t& N_threads) {

		auto temp = Planner_copied_parall::Tree_linked::Init_Battery(root_state, problem_handler, N_threads);
		vector<Planner_copied_parall::Tree_star_linked*> Battery;
		Battery.reserve(N_threads);
		for (size_t k = 0; k < N_threads; k++)
			Battery.emplace_back(new Tree_star_linked(temp[k]));
		linked_buffers<Tree_star::Node2Node_Traj>::bind_buffers(Battery);
		return Battery;

	}



	void Planner_copied_parall::_RRT_basic(const Node_State& start, const Node_State& end) {

		auto T_battery = Tree_linked::Init_Battery(start, this->Handler, this->get_Threads());
		vector<Single_Extension_job> Battery_solver;
		this->Init_Single_Extension_battery(&Battery_solver, this->cast_to_I_Tree<Tree_linked>(T_battery), end);
		Absorber<Tree_linked> Ab;
		Ab.Add_battery(T_battery);
		this->_get_solution(Battery_solver, Ab);

	}

	void Planner_copied_parall::_RRT_bidirectional(const Node_State& start, const Node_State& end) {

		auto T_battery_A = Tree_linked::Init_Battery(start, this->Handler, this->get_Threads());
		auto T_battery_B = Tree_linked::Init_Battery(end, this->Handler, this->get_Threads());
		vector<Bidirectional_Extension_job> Battery_solver;
		this->Init_Bidirectional_Extension_battery( &Battery_solver, this->cast_to_I_Tree<Tree_linked>(T_battery_A), this->cast_to_I_Tree<Tree_linked>(T_battery_B));
		Absorber<Tree_linked> Ab;
		Ab.Add_battery(T_battery_A);
		Ab.Add_battery(T_battery_B);
		this->_get_solution(Battery_solver, Ab);

	}

	void Planner_copied_parall::_RRT_star(const Node_State& start, const Node_State& end) {

		auto T_battery = Tree_star_linked::Init_Battery(start, this->Handler, this->get_Threads());

		vector<Tree_star_linked::Extension_star_linked> Battery_solver;
		Battery_solver.reserve(T_battery.size());
		for (size_t k = 0; k < T_battery.size(); k++)
			Battery_solver.emplace_back(T_battery[k], end, &this->Deterministic_coefficient, &this->Cumulate_sol);

		Absorber<Tree_star_linked> Ab;
		Ab.Add_battery(T_battery);
		this->_get_solution(Battery_solver, Ab);

	}

}