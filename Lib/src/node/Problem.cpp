/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/NodeFactory.h>
#include <Error.h>
using namespace std;

namespace mt::node {
    Node::NodeFactory::NodeFactory(const size_t& X_size, const float& gamma, const bool& traj_symm_flag)
        : StateSpaceSize(X_size)
        , Traj_symmetric(traj_symm_flag)
        , Gamma_coeff(gamma) {
        if(this->StateSpaceSize < 2) {
            throw Error("size of the state space should be at least 2");
        }
        if(this->Gamma_coeff < 0.f) {
            throw Error("gamma coefficient can't be lower than 0");
        }
        this->Steer_max_iteration = 1;
    }

	void Node::I_Node_factory::Cost_to_go_constraints(float* result, const Node* start, const Node* ending_node){

		delete this->last_computed_traj;
		this->Recompute_trajectory_in_cache(start->Get_State() , ending_node->Get_State());
		*result = this->last_computed_traj->Cost_to_go();
		if(*result == FLT_MAX)  return;

		size_t k = 0;
		while(this->last_computed_traj->Advance()){
			if(this->Check_reached_in_cache()) {
				*result = FLT_MAX;
				return;
			}
			++k;
			if(k == this->Cost_to_go_constraints_max_iterations)  {
				*result = FLT_MAX;
				return;
			}
		}
		if (this->Check_reached_in_cache()) {
			*result = FLT_MAX;
			return;
		}

	}

    Node Node::NodeFactory::createRandomNode() {
        Node randNode(this->StateSpaceSize);
        this->createRandomState(randNode.state);
        return randNode;
	};

	Node Node::I_Node_factory::Steer(Node* start, const  Node* trg, bool* trg_reached) {

		*trg_reached = false;

		delete this->last_computed_traj;
		this->Recompute_trajectory_in_cache(start->Get_State() , trg->Get_State());
		float cost2go =  this->last_computed_traj->Cost_to_go();
		if(cost2go == FLT_MAX)  return Node(nullptr);

		const float* steered = nullptr;
		float cost, cost_prev;

		size_t k=0;
		while(true){
			cost_prev = this->last_computed_traj->Cost_to_go_Cumulated();
			if(!this->last_computed_traj->Advance()) {
				if (this->Check_reached_in_cache()) {
					if (k > 0) {
						steered = this->last_computed_traj->Get_state_previous();
						cost = cost_prev;
					}
					break;
				}
				else {
					*trg_reached = true;
					float* temp = this->Alloc_state();
					Array::Array_copy(temp, trg->Get_State(), this->State_size);
					return Node(start, cost2go, temp);
				}
			}

			if (this->Check_reached_in_cache()) {
				if (k > 0) {
					steered = this->last_computed_traj->Get_state_previous();
					cost = cost_prev;
				}
				break;
			}

			if(k == 0) steered = this->last_computed_traj->Get_state_current();
			cost = this->last_computed_traj->Cost_to_go_Cumulated();

			++k;
			if(k == this->Steer_max_iteration) break;
		}
		if(steered == nullptr) return Node(nullptr);
		else{
			float* temp = this->Alloc_state();
			Array::Array_copy(temp , steered, this->State_size);
			return Node(start , cost, temp);
		}

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

	void Node::I_Node_factory::Interpolate(std::list<Array>& waypoints_to_interpolate, float* cost_total) {

		if (waypoints_to_interpolate.size() < 2) throw 0;
		if (cost_total != nullptr) *cost_total = 0.f;

		size_t S = this->Get_State_size();
		auto it_w = waypoints_to_interpolate.begin();
		if (it_w->size() != S) throw 1;
		list<Array>::iterator it_prev;
		++it_w;
		for (it_w = it_w; it_w != waypoints_to_interpolate.end(); ++it_w) {
			if (it_w->size() != S) throw 1;
			it_prev = it_w;
			--it_prev;
			delete this->last_computed_traj;
			this->Recompute_trajectory_in_cache(&(*it_prev)[0], &(*it_w)[0]);
			if (this->last_computed_traj->Cost_to_go() == FLT_MAX) throw 2;
			if (cost_total != nullptr) *cost_total += this->last_computed_traj->Cost_to_go();
			while (this->last_computed_traj->Advance()) 
				waypoints_to_interpolate.insert(it_w , move(Array(this->last_computed_traj->Get_state_current() , S)));
		}

	}
}