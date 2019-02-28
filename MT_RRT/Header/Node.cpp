#include "../header/Node.h"

Node::~Node() {
	free(this->mData.mNames);
};

void Node::Cost_to_root(float* result) {
	*result = 0.f;

#ifdef _DEBUG
	int k = 0;
#endif // _DEBUG
	Node* att_node = this;
	while (att_node->mData.pFather != NULL) {
		*result += att_node->mData.mCost_traj_from_father;
		att_node = att_node->mData.pFather;

#ifdef _DEBUG
		k++;
		if (k > 10000)
			abort();
#endif // _DEBUG
	}

}

void Node::copy_State(VectorF* S) {
	int Size = this->mData.mPose.Get_Size();
	list<VectorF>::iterator it;
	for (it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++)
		Size += it->Get_Size();

	float* content = (float*)malloc(Size * sizeof(float));
	int k;
	for (k = 0; k < this->mData.mPose.Get_Size(); k++)
		content[k] = this->mData.mPose[k];
	int offset = this->mData.mPose.Get_Size();
	for (it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++) {
		k = 0;
		for (k = 0; k < this->mData.mPose.Get_Size(); k++)
			content[k + offset] = (*it)[k];

		offset += it->Get_Size();
	}

	S->copy(content, Size);
}

int	 Node::Get_State_size() {
	int d = this->mData.mPose.Get_Size();
	for (list<VectorF>::iterator it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++)
		d += it->Get_Size();

	return d;
}

list<int> Node::Get_State_subsizes() {
	list<int> dims;
	dims.push_back(this->mData.mPose.Get_Size());
	for (list<VectorF>::iterator it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++)
		dims.push_back(it->Get_Size());

	return dims;
}

Node* Node::copy_this() {
	Node* copia = new Node();

	copia->mData.pFather = this->mData.pFather;
	copia->mData.mPose.copy(this->mData.mPose);
	if (!this->mData.mState_additional.empty()) {
		for (list<VectorF>::iterator it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++) {
			copia->mData.mState_additional.push_back(VectorF());
			copia->mData.mState_additional.back().copy(*it);
		}
	}
	copia->mData.mCost_traj_from_father = this->mData.mCost_traj_from_father;

	return copia;
}

void Node::H_star_search::Set_father(Node* involved, Node* new_fath, float& new_cost_from_father) {
#ifdef _DEBUG
	if (new_fath == involved) {
		cout << "Error in Set_father, new father cannot be this node \n";
		abort(); }
#endif // _DEBUG

	involved->mData.pFather = new_fath;
	involved->mData.mCost_traj_from_father = new_cost_from_father;

#ifdef _DEBUG
	float cost_debug;
	involved->Cost_to_root(&cost_debug);
#endif // _DEBUG
}

void Node::H_MultiP::Get_State_copy(Node* involved, list<float*>* container) {
	container->clear();
	container->push_back(VectorF::dangerous_callback.Get_Q(&involved->mData.mPose));
	if (!involved->mData.mState_additional.empty()) {
		for (list<VectorF>::iterator itV = involved->mData.mState_additional.begin(); itV != involved->mData.mState_additional.end(); itV++)
			container->push_back(VectorF::dangerous_callback.Get_Q(&(*itV)));
	}
}

void Node::Disp_State(ostream& file) {
	int S = this->mData.mPose.Get_Size(), k;
	for (k = 0; k < S; k++)
		file << this->mData.mPose[k] << " ";
	if (!this->mData.mState_additional.empty()) {
		for (list<VectorF>::iterator it = this->mData.mState_additional.begin(); it != this->mData.mState_additional.end(); it++) {
			S = it->Get_Size();
			for (k = 0; k < S; k++)
				file << (*it)[k] << " ";
		}
	}
}





Node* I_Node_Factory::create_from_info(Node* father, list<float*>& state_copy, list<int>& state_size, int* name_copy) {
	Node* new_node = new Node();
	new_node->mData.pFather = father;
	list<float*>::iterator state_copy_iter = state_copy.begin();
	list<int>::iterator    state_size_iter = state_size.begin();
	VectorF::dangerous_callback.set(&new_node->mData.mPose, *state_copy_iter, *state_size_iter);
	state_copy_iter++;
	state_size_iter++;
	for (state_copy_iter; state_copy_iter != state_copy.end(); state_copy_iter++) {
		new_node->mData.mState_additional.push_back(VectorF());
		VectorF::dangerous_callback.set(&new_node->mData.mState_additional.back(), *state_copy_iter, *state_size_iter);
		state_size_iter++;
	}

	if (father != NULL) //is NULL, when computing new roots for mpi_slave_dec, when beginning newer expnasion cyclic
		this->Distance(father, new_node, &new_node->mData.mCost_traj_from_father);

	new_node->mData.mNames = name_copy;

#ifdef _DEBUG
	float cost_debug;
	new_node->Cost_to_root(&cost_debug);
#endif // _DEBUG

	return new_node;
}

void I_Node_Factory::Extract_path(Node* N, list<VectorF>& States) {
	States.clear();

	list<Node*> path;
	path.push_front(N);

	while (path.front()->Get_father() != NULL) {
		path.push_front(path.front()->Get_father());
	}

	list<Node*>::iterator it = path.begin();
	States.push_back(VectorF());
	(*it)->copy_State(&States.front());
	it++;
	for (it; it != path.end(); it++) {
		States.push_back(VectorF());
		(*it)->copy_State(&States.back());
	}
}

void I_Node_Factory::Extract_path(Node* S, Node* E, list<VectorF>& States) {
	this->Extract_path(S, States);

	list<Node*> path;
	path.push_back(E->Get_father());
	while (path.back()->Get_father() != NULL) {
		path.push_back(path.back()->Get_father());
	}

	list<Node*>::iterator it = path.begin();
	list<VectorF>::iterator itt;
	Node* n_prev = E;
	for (it; it != path.end(); it++) {
		States.push_back(VectorF());
		(*it)->copy_State(&States.back());

		n_prev = *it;
	}
}





void Node_Factory_4simulation::Distance(Node* start, Node* ending_node, float* result) {

	for (unsigned int k = 0; k < this->mCoeff_Simul; k++)
		this->pwrapped_factory->Distance(start, ending_node, result);

}

Node* Node_Factory_4simulation::Steer(Node* start, Node* trg, bool* trg_reached) {

	Node* res;
	for (unsigned int k = 1; k < this->mCoeff_Simul; k++) {
		res = this->pwrapped_factory->Steer(start, trg, trg_reached);
		if (res != NULL)
			delete res;
	}
	return this->pwrapped_factory->Steer(start, trg, trg_reached);

}

void Node_Factory_4simulation::Traj_to_target(Node* start, Node* trg, float* result) {

	for (unsigned int k = 0; k < this->mCoeff_Simul; k++) {
		this->pwrapped_factory->Traj_to_target(start, trg, result);
	}

}

I_Node_Factory*	Node_Factory_4simulation::copy() {

	return new Node_Factory_4simulation(this->pwrapped_factory);

}





Node* Path_simple_Node_Factory::New_root(list<VectorF>* State) {
	Node* root = new Node();
	Node::Node_data* root_data = this->Get_data(root);

	list<VectorF>::iterator it_S = State->begin();
	root_data->mPose.copy(*it_S); it_S++;
	for (it_S; it_S != State->end(); it_S++) {
		root_data->mState_additional.push_back(VectorF());
		root_data->mState_additional.back().copy(*it_S); }

	pRob->Set_orientations_byPose(root_data->mPose);
	Obstacles_list* rob_shapes = pRob->Get_shapes();
	if (!rob_shapes->Collision_absent(pObst)) {
		cout << "Error in Path_simple_Node_Factory::New_root, root pose presents collisions \n";
		abort();
	}
	return root;
}

Node* Path_simple_Node_Factory::Random_node() {
	Node* rnd_pose = new Node();
	pRob->Get_random_Pose(&this->Get_data(rnd_pose)->mPose); //within the joints limits

	return rnd_pose;
}

void Path_simple_Node_Factory::Distance(Node* start, Node* ending_node, float* result) {
	*result = sqrtf(this->Get_data(start)->mPose.L2_distance(&this->Get_data(ending_node)->mPose)); //is the lenght of the segment in the joint space conencting start to ending_node
}

Node* Path_simple_Node_Factory::Steer(Node* start, Node* trg, bool* trg_reached) {
	VectorF* pose_trg = &this->Get_data(trg)->mPose;
	VectorF* pose_start = &this->Get_data(start)->mPose;

	Node* steered = new Node();
	Node::Node_data* steered_data = this->Get_data(steered);
	steered_data->pFather = start;

	*trg_reached = false;
	float dist_tot = sqrtf(pose_start->L2_distance(pose_trg));
	if (dist_tot < this->mPasso) { //states distance is lower than the step adopted for checking the collisions: trg is succesfully reached without any further check
		steered_data->mPose.copy(pose_trg);
		this->pRob->Set_orientations_byPose(*pose_trg);
		if (this->pObst->Collision_absent(this->pRob->Get_shapes())) {
			steered_data->mCost_traj_from_father = dist_tot;
			*trg_reached = true;
			return steered;
		}
		else {
			delete steered;
			return NULL;
		}
	}

	VectorF Delta;
	Delta.copy(pose_trg); Delta -= *pose_start;
	Delta *= this->mPasso / dist_tot;
	steered_data->mPose.copy(pose_start);
	steered_data->mPose += Delta;
	steered_data->mCost_traj_from_father = this->mPasso;
	this->pRob->Set_orientations_byPose(steered_data->mPose);
	if (!this->pObst->Collision_absent(this->pRob->Get_shapes())) { //the first steered state is not collision free: the expansion is not possible
		delete steered;
		return NULL;
	}

	// if here at least one state was succesfully steered: the expansion is possible.
	// Try further expansion till reaching trg or find a collision or reach the maximal mExploration_degree 
	if (this->mExploration_degree > 1) {
		VectorF next_pose;
		next_pose.copy(steered_data->mPose);
		float dist_k = this->mPasso;
		for (int k = 1; k < this->mExploration_degree; k++) {
			dist_k += this->mPasso;
			if (dist_k >= dist_tot) {
				this->pRob->Set_orientations_byPose(*pose_trg);
				if (this->pObst->Collision_absent(this->pRob->Get_shapes())) {
					*trg_reached = true;
					steered_data->mPose.copy(pose_trg);
					steered_data->mCost_traj_from_father = dist_tot;
				}
				break;
			}

			next_pose += Delta;
			this->pRob->Set_orientations_byPose(next_pose);
			if (!this->pObst->Collision_absent(this->pRob->Get_shapes()))
				break;

			steered_data->mPose.copy(next_pose);
			steered_data->mCost_traj_from_father += this->mPasso;
		}
	}

	return steered;

}

void Path_simple_Node_Factory::Traj_to_target(Node* start, Node* trg, float* result) {
	VectorF* pose_trg = &this->Get_data(trg)->mPose;
	VectorF* pose_start = &this->Get_data(start)->mPose;


	*result = sqrtf(pose_trg->L2_distance(pose_start));
	if (*result < this->mPasso) //states distance is lower than the step adopted for checking the collisions
		return;

	VectorF temp;
	temp.copy(pose_start);
	VectorF Delta;
	Delta.copy(pose_trg);
	Delta -= *pose_start;
	Delta *= (this->mPasso / *result); //is the incremental step adopted
	float dist_att = 0.f;
	// follow the segment start->trg, considering Delta as step size and checking every time the collisions
	while (true) {
		dist_att += this->mPasso;
		if (dist_att > *result)
			return;
		temp += Delta;

		this->pRob->Set_orientations_byPose(temp);
		if (!this->pObst->Collision_absent(this->pRob->Get_shapes())) {
			//a collsion is detected: return FLOAT_MAX
			*result = FLT_MAX;
			return;
		}
	}

}

I_Node_Factory* Path_simple_Node_Factory::copy() {
	Path_simple_Node_Factory* copia = new Path_simple_Node_Factory(this->pRob->copy(), this->pObst->copy(), this->mPasso, this->mExploration_degree);
	return copia;
}
