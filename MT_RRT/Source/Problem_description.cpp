/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Problem_description.h"
#include <list>
#include <float.h>
using namespace std;

namespace MT_RTT
{

	void Array_copy(float* destination , const float* vals, const size_t& size){ for(size_t k=0; k<size; ++k) destination[k] = vals[k];	}

	Array::Array(const float* vals, const size_t& size) : Array(size){ Array_copy(this->pbuffer , vals, this->Size); }

	Array::Array(const float& val_to_repeat, const size_t& size) : Array(size){ for(size_t k =0; k<this->Size; k++) this->pbuffer[k] = val_to_repeat; }

	Array::Array(const size_t& size) : Size(size) {

		if (size == 0) throw 0;
		this->pbuffer = new float[this->Size];

	}

	Array::Array(const Array& o) : Array(o.Size){ Array_copy(this->pbuffer , o.pbuffer, this->Size); }

	Array& Array::operator=(const Array& o){

		if(o.Size != this->Size) throw 0;
		Array_copy(this->pbuffer , o.pbuffer, this->Size);
		return *this;

	}

	float& Array::operator[](const size_t& pos) {

		if (pos > this->Size) throw 0;
		return this->pbuffer[pos];

	}

	const float& Array::operator[](const size_t& pos) const {

		if (pos > this->Size) throw 0;
		return this->pbuffer[pos];

	}



	Node::Node(Node&& o) {

		this->Father = o.Father;
		this->Cost_traj_from_father = o.Cost_traj_from_father;
		this->State = o.State;
		o.State = nullptr;
		o.Father = nullptr;

	}

	void Node::Cost_to_root(float* result, const size_t& I_max) const {

		*result = 0.f;

		size_t k = 0;
		const Node* att_node = this;
		while (att_node != nullptr) {
			++k;
			if (k == I_max) throw 0;
			* result += att_node->Cost_traj_from_father;
			att_node = att_node->Father;
		}

	}

	void Node::Cost_to_root(float* result) const {

		*result = 0.f;

		const Node* att_node = this;
		while (att_node != nullptr) {
			* result += att_node->Cost_traj_from_father;
			att_node = att_node->Father;
		}

	};

	Node Node::I_Node_factory::Random_node() {

		auto random_state = this->Alloc_state();
		this->Random_node(random_state);
		return Node(random_state);

	};

	Node Node::I_Node_factory::Steer(Node* start, const  Node* trg, bool* trg_reached) {

		auto steered_state = this->Alloc_state();
		float cost;
		this->Steer(&cost, steered_state, start->Get_State(), trg->Get_State(), trg_reached);
		if (cost == FLT_MAX) {
			delete[] steered_state;
			*trg_reached = false;
			return Node(nullptr);
		}
		else 
			return Node(start, cost, steered_state);

	};

	Node Node::I_Node_factory::Clone_Node(const Node& o) {

		const float* state = o.Get_State();
		size_t S = this->Get_State_size();
		float* state_clone = this->Alloc_state();
		for (size_t k = 0; k < S; ++k) state_clone[k] = state[k];
		return Node(o.Get_Father(), o.Get_Cost_from_father(), state_clone);

	}

	Node Node::I_Node_factory::New_root(const Array& state) {

		size_t S = this->Get_State_size();
		if (state.size() != S) throw 0;

		float* state_cloned = this->Alloc_state();
		for (size_t k = 0; k < S; ++k) state_cloned[k] = state[k];

		return Node(state_cloned);

	};

	float* Node::I_Node_factory::Alloc_state() { return new float[this->Get_State_size()]; }




	Node_factory_multiple_steer::Node_factory_multiple_steer(std::unique_ptr<I_Node_factory>& to_wrap, const size_t& max_numb_trials): I_Node_factory_decorator(to_wrap) {

		this->Maximum_trial = max_numb_trials;
		if (this->Maximum_trial < 2) throw 0;

		this->Gamma_multiple = this->Get_Wrapped()->Get_Gamma() * this->Maximum_trial;

	};

	std::unique_ptr<Node::I_Node_factory>		Node_factory_multiple_steer::copy() {

		unique_ptr<I_Node_factory> temp = this->Get_Wrapped()->copy();
		return std::unique_ptr<I_Node_factory>(new Node_factory_multiple_steer(temp, this->Maximum_trial)); 
	
	};

	void Node_factory_multiple_steer::Steer(float* cost_steered, float* steered_state, const float* start_state, const float* target_state, bool* trg_reached) {

		list<float*> steered;
		float delta_cost;
		const float* to_steer = start_state;
		*cost_steered = 0.f;
		for (size_t k = 0; k < this->Maximum_trial; ++k) {
			steered.push_back(this->Alloc_state());
			this->Get_Wrapped()->Steer(&delta_cost, steered.back(), to_steer, target_state, trg_reached);

			if (delta_cost == FLT_MAX) {
				delete[] steered.back();
				steered.pop_back();
				break;
			}

			*cost_steered += delta_cost;

			if (*trg_reached)
				break;

			to_steer = steered.back();
		}

		if (steered.empty()) 
			*cost_steered = FLT_MAX;
		else {
			size_t k, K=this->Get_State_size();
			for (k = 0; k < K; ++k)
				steered_state[k] = steered.back()[k];
			auto it_end = steered.end();
			for (auto it = steered.begin(); it != it_end; ++it)
				delete[] *it;
		}

	}

}