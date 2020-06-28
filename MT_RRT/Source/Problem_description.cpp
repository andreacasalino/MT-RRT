/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../Header/Problem_description.h"
#include <list>
#include <float.h>
#include <math.h>
using namespace std;

namespace MT_RTT
{

	void Array::Array_copy(float* destination , const float* vals, const size_t& size){ for(size_t k=0; k<size; ++k) destination[k] = vals[k];	}

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

	void Node::I_Node_factory::Cost_to_go(float* result, const Node* start, const Node* ending_node){

		delete this->last_computed_traj;
		this->Recompute_trajectory_in_cache(start->Get_State() , ending_node->Get_State());
		*result = this->last_computed_traj->Cost_to_go();

	}

	void Node::I_Node_factory::Cost_to_go_constraints(float* result, const Node* start, const Node* ending_node){

		delete this->last_computed_traj;
		this->Recompute_trajectory_in_cache(start->Get_State() , ending_node->Get_State());
		*result = this->last_computed_traj->Cost_to_go();
		if(*result == FLT_MAX)  return;
		while(this->last_computed_traj->Advance()){
			if(this->Check_reached_in_cache()) {
				*result = FLT_MAX;
				return;
			}
		}
		if (this->Check_reached_in_cache()) {
			*result = FLT_MAX;
			return;
		}

	}

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

	Node::I_Node_factory::Composite_trajectory::Composite_trajectory(const float* start, const float* end, I_Node_factory* caller) :
		I_trajectory(start, end, caller), Pieces_not_initialized(true), increment_Pieces_it(false), Cumulated_from_Pieces_prev_cost(0.f) { };

	void Node::I_Node_factory::Composite_trajectory::Init_Pieces(const std::list<I_trajectory*>& pieces, const std::list<float*>& waypoints_to_save) {

		if (!this->Pieces_not_initialized) throw 0;

		this->Pieces = pieces;
		this->Waypoints = waypoints_to_save;
		this->Pieces_it = this->Pieces.begin();

		this->Pieces_not_initialized = false;

	}

	Node::I_Node_factory::Composite_trajectory::~Composite_trajectory(){

		this->Cursor_along_traj = nullptr;
		this->Cursor_previous = nullptr;

		for (auto it = this->Pieces.begin(); it != this->Pieces.end(); ++it) delete* it;
		for (auto it = this->Waypoints.begin(); it != this->Waypoints.end(); ++it) delete* it;

	}

	float Node::I_Node_factory::Composite_trajectory::Cost_to_go(){

		if (this->Pieces_not_initialized) throw 0;
		if (this->Pieces.empty()) return FLT_MAX;

		auto it = this->Pieces.begin();
		float cost = (*it)->Cost_to_go();
		if (cost == FLT_MAX) return FLT_MAX;
		++it;
		for (it = it; it != this->Pieces.end(); ++it) {
			cost += (*it)->Cost_to_go();
			if (cost == FLT_MAX) return FLT_MAX;
		}
		return cost;
 
	}

	bool Node::I_Node_factory::Composite_trajectory::Advance() {

		if (this->Pieces_not_initialized) throw 0;

		if (this->increment_Pieces_it) {
			this->Cumulated_from_Pieces_prev_cost = this->Cumulated_cost;
			++this->Pieces_it;
			this->increment_Pieces_it = false;
		}

		if (!(*this->Pieces_it)->Advance())  {
			auto temp = this->Pieces_it;
			++temp;
			if (temp == this->Pieces.end()) {
				this->Cumulated_cost = this->Cumulated_from_Pieces_prev_cost + (*this->Pieces_it)->Cost_to_go_Cumulated();
				this->Cursor_along_traj = Get_state_current_another(*this->Pieces_it);
				this->Cursor_previous = Get_state_previous_another(*this->Pieces_it);
				return false;
			}
			this->increment_Pieces_it = true;
		}
		this->Cumulated_cost = this->Cumulated_from_Pieces_prev_cost + (*this->Pieces_it)->Cost_to_go_Cumulated();
		this->Cursor_along_traj = Get_state_current_another(*this->Pieces_it);
		this->Cursor_previous = Get_state_previous_another(*this->Pieces_it);
		return true;

	}




	float Equispaced_Node_factory::linear_trajectory::Euclidean_distance(const float* p1, const float* p2, const size_t& Size) {

		float res = 0.f;
		for (size_t k = 0; k < Size; ++k) res += (p1[k] - p2[k]) * (p1[k] - p2[k]);
		res = sqrtf(res);
		return res;

	}

	float Equispaced_Node_factory::linear_trajectory::Cost_to_go(){

		return Euclidean_distance(this->Start , this->End, this->Caller->Get_State_size());

	}

	bool Equispaced_Node_factory::linear_trajectory::Advance(){

		size_t K = this->Caller->Get_State_size(), k;
		if(this->Cursor_along_traj == nullptr){
			this->step_max = 1;
			this->step = 0;
			size_t temp;
			float* steer_degree = &dynamic_cast<Equispaced_Node_factory*>(this->Caller)->Steer_degree;
			for(k=0;k<K; ++k){
				temp = (size_t)ceilf(abs(this->Start[k] - this->End[k]) / *steer_degree);
				if(temp > this->step_max) this->step_max = temp;
			}
			float coeff = 1.f / (float)this->step_max;
			this->Delta = new float[K];
			this->Cursor_along_traj = new float[K];
			for (k = 0; k < K; ++k) {
				this->Delta[k] = coeff * (this->End[k] - this->Start[k]);
				this->Cursor_along_traj[k] = this->Start[k] + this->Delta[k];
			}

			this->Delta_norm = 0.f;
			for(k=0;k<K; ++k) this->Delta_norm += this->Delta[k] * this->Delta[k];
			this->Delta_norm = sqrtf(this->Delta_norm);
			this->Cumulated_cost = this->Delta_norm;

			this->Cursor_previous = new float[K];
			Array::Array_copy( this->Cursor_previous, this->Start, K);
		}
		else{
			float* temp = this->Cursor_previous;
			this->Cursor_previous = this->Cursor_along_traj;
			this->Cursor_along_traj = temp;
			for(k=0;k<K; ++k) 
				this->Cursor_along_traj[k] = this->Cursor_previous[k] + this->Delta[k];
			this->Cumulated_cost += this->Delta_norm;
		}
		++this->step;

		return (this->step < this->step_max);

	}

	class Linear_Equispaced_factory : public Equispaced_Node_factory {
	public:
		Linear_Equispaced_factory(const size_t& X_size, const float& steer_degree) : Equispaced_Node_factory(X_size, 0.f, steer_degree, true){};
	private:
		virtual std::unique_ptr<I_Node_factory>			copy() { return std::unique_ptr<I_Node_factory>(); };
		virtual void									Random_node(float* random_state) { };
		virtual bool									Check_reached_in_cache() { return false; };
	};
	void Equispaced_Node_factory::Interpolate_linear_eqauispaced(std::list<Array>& waypoints_to_interpolate, const float& steer_degree) {

		if (waypoints_to_interpolate.empty()) throw 0;
		Linear_Equispaced_factory temp(waypoints_to_interpolate.front().size() , steer_degree);
		temp.I_Node_factory::Interpolate(waypoints_to_interpolate);

	}

}