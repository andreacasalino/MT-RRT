/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Tree.h"
#include <float.h>
#include <cmath>
#include "../Header/json.h"
using namespace std;

#ifdef _REW_DEBUG
#include <fstream>
#endif

namespace MT_RTT
{

	const Node* I_Tree::Extend_random() {

		this->Get_extend_info()->random_or_deter = true;
		Node rand_state = this->Get_Problem_Handler()->Random_node();
		return this->Extend(&rand_state);

	};

	const Node* I_Tree::Extend_deterministic(const Node* target) {

		this->Get_extend_info()->random_or_deter = false;
		return this->Extend(target);

	}

	bool I_Tree::Extend_reached_determ_target() {

		auto info = this->Get_extend_info();
		if ((!info->random_or_deter) && (info->target_reached))
			return true;
		else
			return false;

	}

	string I_Tree::Get_Tree_as_JSON() {

		auto nodes = this->Get_Nodes();
		size_t State_size = this->Get_Problem_Handler()->Get_State_size();
		string JSON = "[\n";
		auto it = nodes->begin();
		JSON += "{\"E\":";
		JSON += json_parser::load_JSON(&(*it)->Get_State()[0], State_size);
		JSON += ",\"S\":";
		JSON += json_parser::load_JSON(&(*it)->Get_State()[0], State_size);
		JSON += "}\n";
		++it;
		auto it_end = nodes->end();
		while (it != it_end) {
			JSON += ",{\"E\":";
			JSON += json_parser::load_JSON(&(*it)->Get_State()[0], State_size);
			JSON += ",\"S\":";
			JSON += json_parser::load_JSON(&(*it)->Get_Father()->Get_State()[0], State_size);
			JSON += "}\n";
			++it;
		}
		JSON += "]";
		return JSON;

	}



	Tree_concrete::Tree_concrete(Node::I_Node_factory* handler, const bool& clone_handler) : Problem_handler_was_cloned(clone_handler) {

		if (clone_handler) {
			auto temp = handler->copy();
			this->Problem_handler = temp.get();
			temp.release();
		}
		else
			this->Problem_handler = handler;
		this->__ext_info.target_reached = false;
		this->__ext_info.random_or_deter = true;

	};

	Tree_concrete::Tree_concrete(const Array& root_state, Node::I_Node_factory* handler, const bool& clone_handler ) :
	Tree_concrete(handler, clone_handler) {

		this->Nodes.push_back(new Node(handler->New_root(root_state)));

	};

	Tree_concrete::~Tree_concrete() {

		if (this->Problem_handler_was_cloned)  delete this->Problem_handler;
		auto it_end = this->Nodes.end();
		for (auto it = this->Nodes.begin(); it != it_end; ++it) 
			delete* it;
		this->Nodes.clear();

	};

	Node*	Tree_concrete::Nearest_Neighbour(const Node* state) {

		auto Nodes = this->Get_Nodes();
		size_t Nodes_size = this->__get_Nodes_size(); //done in a separate function to have a critical region
		auto Problem = this->Get_Problem_Handler();
		auto it_N = Nodes->begin();
		Node* nearest = *it_N;
		float dist_min, dist_att;
		Problem->Cost_to_go(&dist_min, *it_N, state);
		++it_N;
		for (size_t k = 1; k < Nodes_size; ++k) {
			Problem->Cost_to_go(&dist_att, *it_N, state);
			if (dist_att < dist_min) {
				dist_min = dist_att;
				nearest = *it_N;
			}
			++it_N;
		}
		return nearest;

	};

	Node*	Tree_concrete::Extend(const Node* target) {

		Node* near_N = this->Nearest_Neighbour(target);
		Node* steered = new Node(this->Get_Problem_Handler()->Steer(near_N, target, &this->__ext_info.target_reached));
		if (steered->Get_State() == nullptr) {
			delete steered;
			return nullptr;
		}

		if (this->__ext_info.target_reached && (!this->__ext_info.random_or_deter)) {
			delete steered;
			return near_N;
		}

		this->Nodes.push_back(steered);
		return steered;

	};



	void Tree_star::Near_set(std::list<Node*>* near_set, const Node* state) {

		auto Nodes = this->Get_Nodes();
		auto Problem = this->Get_Problem_Handler();
		Node* Nearest_neigh = state->Get_Father();
		float Tree_size = (float)Nodes->size();

		float ray = Problem->Get_Gamma() * powf(logf(Tree_size) / Tree_size, 1.0f / (float)Problem->Get_State_size());
		near_set->emplace_back(Nearest_neigh);
		float dist_att;
		auto itN = Nodes->begin();
		while (true) {
			if (*itN == state)
				break;
			if (*itN != Nearest_neigh) {
				Problem->Cost_to_go(&dist_att, *itN, state);
				if (dist_att <= ray)
					near_set->emplace_back(*itN);
			}
			++itN;
		}

	};

	Node*	Tree_star::Extend(const Node* target) {

		Node* added = this->I_Tree_decorator::Extend(target);
		if (added != nullptr) {
			if (this->Extend_reached_determ_target()) return added;

			std::list<Node2Node_Traj> rewird_to_do;
			this->Connect_to_best_Father_and_eval_Rewirds(&rewird_to_do, added);
			auto it_end = rewird_to_do.end();
			for (auto it = rewird_to_do.begin(); it != it_end; ++it)
				it->end->Set_Father(it->start , it->cost);
		}
		return added;

	};

	void Tree_star::Connect_to_best_Father_and_eval_Rewirds(std::list<Node2Node_Traj>* possible_rewirds, Node* last_added) {

		possible_rewirds->clear();
		auto Problem = this->Get_Problem_Handler();
		std::list<Node*> Near_set;
		this->Near_set(&Near_set, last_added); // near set of the last element added
		if (Near_set.size() == 1)
			return;

		float c_min, c_att;
		auto it_Near = Near_set.begin();

#ifdef _REW_DEBUG
		size_t T_size = this->Get_Nodes()->size();
		try {
			(*it_Near)->Cost_to_root(&c_min, T_size + 5);
		}
		catch (const int&) {
			this->Print_Debug(*it_Near);
		}
#else
		(*it_Near)->Cost_to_root(&c_min);
#endif
		possible_rewirds->emplace_back();
		possible_rewirds->back().start = *it_Near;
		possible_rewirds->back().end = last_added;
		possible_rewirds->back().cost = last_added->Get_Cost_from_father();
		c_min += possible_rewirds->back().cost;
		++it_Near;
		auto best_traj = possible_rewirds->begin();
		while (it_Near != Near_set.end()){
			possible_rewirds->emplace_back();
			possible_rewirds->back().start = *it_Near;
			possible_rewirds->back().end = last_added;
			Problem->Cost_to_go_constraints(&possible_rewirds->back().cost, *it_Near, last_added);
			if (possible_rewirds->back().cost != FLT_MAX) {
#ifdef _REW_DEBUG
				try {
					(*it_Near)->Cost_to_root(&c_att, T_size + 5);
				}
				catch (const int&) {
					this->Print_Debug(*it_Near);
				}
#else
				(*it_Near)->Cost_to_root(&c_att);
#endif
				c_att += possible_rewirds->back().cost;
				if (c_att < c_min) {
					c_min = c_att;
					best_traj = possible_rewirds->end();
					best_traj--;
				}
			}
			else possible_rewirds->pop_back();
			++it_Near;
		}

		if (best_traj != possible_rewirds->begin()) {
			best_traj->end->Set_Father(best_traj->start, best_traj->cost);
		}
		possible_rewirds->erase(best_traj);

	// check for rewird
		float cost_to_root_last = c_min;
		float cost_to_root_candidate;
		auto it_traj = possible_rewirds->begin();
		Node* temp_4_shamble;
		while (it_traj != possible_rewirds->end()) {
			if (it_traj->start->Get_Father() != nullptr) {//root node cannot be rewired
				temp_4_shamble = it_traj->start;
				it_traj->start = it_traj->end;
				it_traj->end = temp_4_shamble;
				if (!Problem->Get_symm_flag())
					Problem->Cost_to_go_constraints(&it_traj->cost, it_traj->start, it_traj->end);

				if (it_traj->cost != FLT_MAX) {
					it_traj->end->Cost_to_root(&cost_to_root_candidate);
					if (cost_to_root_candidate < (it_traj->cost + cost_to_root_last))
						it_traj = possible_rewirds->erase(it_traj);
					else
						++it_traj;
				}
				else it_traj = possible_rewirds->erase(it_traj);
			}
			else it_traj = possible_rewirds->erase(it_traj);
		}

	};

#ifdef _REW_DEBUG
	void Tree_star::Print_Debug(Node* to_highlight) {

		string temp = this->Get_Tree_as_JSON();
		auto state = to_highlight->Get_State();
		ofstream f("__temp_tree");
		f << "{\"Tree\":\n";
		f << temp;
		f << ",\"Node_involved\":";
		f << json_parser::load_JSON(&state[0], this->Get_Problem_Handler()->Get_State_size());
		f << "}";
		f.close();
		abort();

	}
#endif // DEBUG

}