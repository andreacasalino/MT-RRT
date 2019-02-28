#include "Node_Control.h"





void addition_Matrix_x_Vector(VectorF* result, VectorF& M, VectorF& V) { //result = result + M*V 

#ifdef _DEBUG
	if (M.Get_Size() != (V.Get_Size() * V.Get_Size())) abort();
#endif // _DEBUG


	int k = 0, c;
	for (int r = 0; r < result->Get_Size(); r++) {
		for (c = 0; c < result->Get_Size(); c++) {
			result->operator[](r) += M[k] * V[c];
			k++;
		}
	}

}

void subtraction_Matrix_x_Vector(VectorF* result, VectorF& M, VectorF& V) { //result = result - M*V 

#ifdef _DEBUG
	if (M.Get_Size() != (V.Get_Size() * V.Get_Size())) abort();
#endif // _DEBUG

	int k = 0, c;
	for (int r = 0; r < result->Get_Size(); r++) {
		for (c = 0; c < result->Get_Size(); c++) {
			result->operator[](r) -= M[k] * V[c];
			k++;
		}
	}

}

void LQR(VectorF* Gain, const int& n,  VectorF& A, float* T_close_loop) { // u = Gain*x; closed loop system de=(A+Gain)*e

#ifdef _DEBUG
	if (A.Get_Size() != (n * n)) abort();
#endif // _DEBUG

	MatrixXf Hamilt(2 * n, 2 * n);
	Hamilt.setZero();
	Hamilt.block(0, n, n, n) = MatrixXf::Identity(n, n);
	Hamilt.block(n, 0, n, n) = MatrixXf::Identity(n, n);
	int c , k = 0; 
	for (int r = 0; r < n; r++) {
		for (c = 0; c < n; c++) {
			Hamilt( r, c) = A[k];
			Hamilt(n + r, n + c) = -A[c*n + r]; //it's A transpose
			k++;
		}
	} 
	//cout << Hamilt << endl << endl;

	EigenSolver<MatrixXf> eig_solver(Hamilt);
	VectorXcf poles = eig_solver.eigenvalues();
	list<int> stable_pos;
	for (int k = 0; k < (int)(2 * n); k++) {
		if (poles(k).real() <= 0.f) {
			stable_pos.push_back(k);

			//cout << poles(k) << endl;
		}
	}

	if ((int)stable_pos.size() != n) abort();

	MatrixXcf U(eig_solver.eigenvectors());

	MatrixXcf U1(n, n), U2(n, n);
	list<int>::iterator it_stable;
	int kc;
	for (k = 0; k < (int)n; k++) {
		kc = 0;
		for (it_stable = stable_pos.begin(); it_stable != stable_pos.end(); it_stable++) {
			U1(k, kc) = U(k, *it_stable);
			kc++;
		}
	}
	for (k = 0; k < (int)n; k++) {
		kc = 0;
		for (it_stable = stable_pos.begin(); it_stable != stable_pos.end(); it_stable++) {
			U2(k, kc) = U(k + (int)n, *it_stable);
			kc++;
		}
	}

	U1 = U1.inverse();

	MatrixXf Gain_mat = (U2 * U1).real();
	k = 0;
	float* Gain_f = (float*)malloc(n * n * sizeof(float));
	for (int r = 0; r < n; r++) {
		for (c = 0; c < n; c++) {
			Gain_f[k] = Gain_mat(r, c);
			k++;
		}
	}
	Gain->copy(Gain_f, n*n);

	EigenSolver<MatrixXf> eig_closed_solver(Hamilt.block(0, 0, n, n) + Gain_mat);
	VectorXcf poles_closed = eig_closed_solver.eigenvalues();
	*T_close_loop = FLT_MAX;
	float temp;
	for (k = 0; k < n; k++) {
		temp = poles_closed(k).real()*poles_closed(k).real() + poles_closed(k).imag()*poles_closed(k).imag();
		if ( temp < *T_close_loop ) *T_close_loop = temp;
	}
	*T_close_loop = sqrtf(temp);
	*T_close_loop = 6.5f / *T_close_loop;

}

float* extract_as_vector(MatrixXf& M) {

	float* V = (float*)malloc(M.rows()* M.cols() * sizeof(float));
	int c, k=0;
	for (int r = 0; r < (int)M.rows(); r++) {
		for (c = 0; c < (int)M.cols(); c++) {
			V[k] = M(r, c);
			k++;
		}
	}
	return V;

}
I_Dynamics_function::Constraint::Constraint(MatrixXf& alfa, MatrixXf& beta, MatrixXf& gamma):
	Alfa(extract_as_vector(alfa), (int)alfa.rows() *(int)alfa.cols()), 
	Beta(extract_as_vector(beta), (int)beta.rows() *(int)beta.cols()),
	Gamma(extract_as_vector(gamma), (int)gamma.rows() *(int)gamma.cols()) {

};





I_Dynamics_function::I_Dynamics_function(VectorF& xmin, VectorF& xmax) : pCnstr(NULL) {

	if (xmin.Get_Size() != xmax.Get_Size()) abort();

	this->x_min.copy(xmin);
	this->x_delta.copy(xmax);
	this->x_delta -= this->x_min;

}

I_Dynamics_function::I_Dynamics_function(VectorF& xmin, VectorF& xmax, Constraint& Cnstr):
	I_Dynamics_function(xmin, xmax) {

	this->pCnstr = new Constraint(Cnstr);
	this->pCnstr->Alfa.copy(Cnstr.Alfa);
	this->pCnstr->Beta.copy(Cnstr.Beta);
	this->pCnstr->Gamma.copy(Cnstr.Gamma);
	//consistency check
	int N_cnstr = Cnstr.Gamma.Get_Size();
	int x_size = xmin.Get_Size();
	if ((x_size*N_cnstr) != Cnstr.Alfa.Get_Size()) abort();
	if ((x_size*N_cnstr) != Cnstr.Beta.Get_Size()) abort();

}

I_Dynamics_function::I_Dynamics_function(I_Dynamics_function* to_copy) {

	if (to_copy->pCnstr == NULL) this->pCnstr = NULL;
	else {
		this->pCnstr = new Constraint(*to_copy->pCnstr);
	}
	this->x_min.copy(to_copy->x_min); 
	this->x_delta.copy(to_copy->x_delta);

}

void I_Dynamics_function::Get_random_state(VectorF* x_rand) {

	float* x_rnd = (float*)malloc(this->x_min.Get_Size() * sizeof(float));
	for (int k = 0; k < this->x_min.Get_Size(); k++)
		x_rnd[k] = this->x_delta[k] * (float)rand() / (float)RAND_MAX + this->x_min[k];
	x_rand->copy(x_rnd, this->x_min.Get_Size());

}

bool I_Dynamics_function::is_admitted(VectorF& x, VectorF& u) {

	if (this->pCnstr == NULL) return true;

	int x_size = this->x_delta.Get_Size(), k;
	VectorF temp(0.f, x_size);

	addition_Matrix_x_Vector(&temp, this->pCnstr->Alfa, x);
	addition_Matrix_x_Vector(&temp, this->pCnstr->Beta, u);

	for (k = 0; k < x_size; k++) {
		if (temp[k] > this->pCnstr->Gamma[k]) return false;
	}
	return true;

}

void I_Dynamics_function::H_min_max::Get_extremals(I_Dynamics_function* to_extract, VectorF* xmin, VectorF* xmax) {

	xmin->copy(to_extract->x_min);
	xmax->copy(to_extract->x_delta);
	xmax->operator+=(*xmin);

}





void Non_Linear_system::Init(string& system_data) {

	vector<vector<float>> data_raw = XML_reader::Load(system_data);

	auto n = data_raw.front().size();
	if ((data_raw.size() - 2) != (2 * n + 1)) abort();

	this->x_min.Resize((int)n);
	this->x_delta.Resize((int)n);
	for (int k = 0; k < n; k++) {
		this->x_min[k] = data_raw[0][k];
		this->x_delta[k] = data_raw[1][k];
	}
	this->x_delta -= this->x_min;

	this->exp_coeff.Resize((int)n);
	this->T.Resize((int)(n*n));
	this->invT.Resize((int)(n*n));

	int kc;
	for (int k = 0; k < (int)n; k++) {
		this->exp_coeff[k] = data_raw[2][k];
		for (kc = 0; kc < (int)n; kc++) {
			this->T[k*(int)n + kc] = data_raw[k + 2][kc];
			this->invT[k*(int)n + kc] = data_raw[k + 2 + n][kc];
		}
	}

}

Non_Linear_system::Non_Linear_system(string& system_data, VectorF& xmin, VectorF& xmax, Constraint& Cnstr) :
	I_Dynamics_function(xmin, xmax, Cnstr) {

	this->Init(system_data);
};

Non_Linear_system::Non_Linear_system(string& system_data, VectorF& xmin, VectorF& xmax) :
	I_Dynamics_function(xmin, xmax) {

	this->Init(system_data);
};

Non_Linear_system::Non_Linear_system(VectorF& xmin, VectorF& xmax, VectorF& coeff, VectorF& t, VectorF& inv_t) :
	I_Dynamics_function(xmin, xmax) {
	exp_coeff.copy(coeff);
	T.copy(t);
	invT.copy(inv_t);
};

I_Dynamics_function* Non_Linear_system::copy() {
	VectorF temp;
	temp.copy(this->x_min);
	temp += this->x_delta;
	return new Non_Linear_system(this->x_min, temp, exp_coeff, T, invT);
};

void Non_Linear_system::eval_f(VectorF* f, VectorF& x) {

	VectorF temp;
	temp.Resize(this->exp_coeff.Get_Size());
	temp.set_zero();
	addition_Matrix_x_Vector(&temp, this->T, x);
	int k;
	for (k = 0; k < this->exp_coeff.Get_Size(); k++)
		temp[k] *= -1.f - expf(-this->exp_coeff[k] * x[k] * x[k]);

	f->set_zero();
	addition_Matrix_x_Vector(f, this->invT, temp);

};

void Non_Linear_system::eval_der_f(VectorF* A_linearized, VectorF& x) {

	int n = exp_coeff.Get_Size();
	VectorF TX;
	TX.Resize(n);
	TX.set_zero();
	addition_Matrix_x_Vector(&TX, this->T, x);
	int k, k2;

	VectorF temp;
	temp.Resize(n);
	for (k = 0; k < n; k++)
		temp[k] = this->exp_coeff[k] * expf(-this->exp_coeff[k] * x[k] * x[k]) * TX[k];

	A_linearized->set_zero();
	for (k = 0; k < n; k++) {
		for (k2 = 0; k2 < n; k2++)
			A_linearized->operator[](k*n + k2) += 2.f * this->invT[k*n + k2] * temp[k2];
	}

	for (k = 0; k < n; k++)
		A_linearized->operator[](k*n + k) += -1.f - expf(-this->exp_coeff[k] * x[k] * x[k]);

};




SystemState_Factory::SystemState_Factory(I_Dynamics_function* motion_equation, const float& dt, const unsigned int& explor_degree, const float& toll) :
	pDyn_Syst(motion_equation->copy()), mDt(dt), mExplor_degree(explor_degree), mToll(toll*toll) { };

I_Node_Factory*	SystemState_Factory::copy() {

	SystemState_Factory* temp = new SystemState_Factory();
	temp->pDyn_Syst = this->pDyn_Syst->copy();
	temp->mDt = this->mDt;
	temp->mExplor_degree = this->mExplor_degree;
	temp->mToll = this->mToll;
	return temp;

}

float SystemState_Factory::Get_Gamma() { 

	Node* N1, *N2;
	list<VectorF> X1, X2; 
	X1.push_back(VectorF()); X2.push_back(VectorF());
	I_Dynamics_function::H_min_max::Get_extremals(this->pDyn_Syst, &X1.back(), &X2.back());

	N1 = this->New_root(&X1);
	N2 = this->New_root(&X2);

	float gamma;
	this->Distance(N1, N2, &gamma);
	delete N1, N2;

	return 100.f * gamma;

}

Node* SystemState_Factory::New_root(list<VectorF>* State) {

	Node* root = new Node();
	Node::Node_data* root_data = this->Get_data(root);

	root_data->mPose.copy(State->front());

	root_data->mState_additional.push_back(VectorF());
	VectorF A_lin; A_lin.Resize(this->pDyn_Syst->state_size()*this->pDyn_Syst->state_size());
	this->pDyn_Syst->eval_der_f(&A_lin, State->front());
	float* temp_m = (float*)malloc(sizeof(float));
	LQR(&root_data->mState_additional.front(), root_data->mPose.Get_Size(), A_lin, temp_m);
	root_data->mState_additional.push_back(VectorF());
	root_data->mState_additional.back().copy(temp_m, 1);

	return root;
}

Node* SystemState_Factory::Random_node() {

	Node* rnd_node = new Node();
	this->pDyn_Syst->Get_random_state(&this->Get_data(rnd_node)->mPose);
	return rnd_node;

}

void SystemState_Factory::Distance(Node* start, Node* ending_node, float* result) {

	VectorF* xo = &this->Get_data(start)->mPose;
	VectorF* G = &this->Get_data(start)->mState_additional.front();
	VectorF* xf = &this->Get_data(ending_node)->mPose;

	//(xo-xf)'*S*(xo-xf)
	*result = 0.f;
	float partial;
	int k = 0, c;
	for (int r = 0; r < xo->Get_Size(); r++) {
		partial = 0.f;
		for (c = 0; c < xo->Get_Size(); c++) {
			partial += G->operator[](k)*(xo->operator[](c) - xf->operator[](c));
			k++;
		}
		*result += partial * (xo->operator[](r) - xf->operator[](r));
	}

}

// u=A_linearized*(x_o - x_trg) - f_o + G*(x-x_trg) = f_const + G*(x-x_trg);
// dx=f(x) + u
void compute_u(VectorF* u, VectorF& f_const, VectorF& Gain,  VectorF& x, VectorF& x_trg) {

	VectorF err; 
	err.copy(x);
	err -= x_trg;

	u->copy(f_const);
	addition_Matrix_x_Vector(u, Gain, err);

}

void SystemState_Factory::compute_next_state(VectorF* state_attual, VectorF& dx, VectorF& u) {

	this->pDyn_Syst->eval_f(&dx, *state_attual);
	dx += u;
	dx *= this->mDt;
	*state_attual += dx;

}

Node* SystemState_Factory::Steer(Node* start, Node* trg, bool* trg_reached) {
	Node::Node_data* start_data = this->Get_data(start);
	VectorF* xo = &start_data->mPose;
	VectorF* xf = &this->Get_data(trg)->mPose;

	*trg_reached = false;

	VectorF A_lin; A_lin.Resize(this->pDyn_Syst->state_size()*this->pDyn_Syst->state_size());
	this->pDyn_Syst->eval_der_f(&A_lin, *xo);
	VectorF state_attual, u_attual, dx;
	VectorF dx_attual; dx_attual.Resize(this->pDyn_Syst->state_size());
	VectorF f_const; f_const.Resize(this->pDyn_Syst->state_size());

	this->pDyn_Syst->eval_f(&f_const, *xo);
	f_const *= -1.f;
	VectorF err_temp; err_temp.copy(*xo); err_temp -= *xf;
	addition_Matrix_x_Vector(&f_const, A_lin, err_temp);

	state_attual.copy(*xo);

	//first extension
	compute_u(&u_attual, f_const, start_data->mState_additional.front(), state_attual, *xf);
	compute_next_state(&state_attual, dx_attual, u_attual);
	if (!this->pDyn_Syst->is_admitted(state_attual, u_attual)) return NULL;
	VectorF last_valid_state;
	last_valid_state.copy(state_attual);
	for (unsigned int k = 1; k < this->mExplor_degree; k++) {
		if (state_attual.L2_distance(xf) <= this->mToll) {
			*trg_reached = true;
			break;
		}

		compute_u(&u_attual, f_const, start_data->mState_additional.front(), state_attual, *xf);
		compute_next_state(&state_attual, dx_attual, u_attual);
		if (!this->pDyn_Syst->is_admitted(state_attual, u_attual)) break;

		last_valid_state.copy(state_attual);
	}

	Node* steered = new Node();
	Node::Node_data* steered_data = this->Get_data(steered);

	steered_data->pFather = start;
	steered_data->mPose.copy(last_valid_state);

	steered_data->mState_additional.push_back(VectorF());
	this->pDyn_Syst->eval_der_f(&A_lin, last_valid_state);
	float* temp_m = (float*)malloc(sizeof(float));
	LQR(&steered_data->mState_additional.front(), steered_data->mPose.Get_Size() , A_lin, temp_m);
	steered_data->mState_additional.push_back(VectorF());
	steered_data->mState_additional.back().copy(temp_m, 1);

	this->Distance(start, steered, &steered_data->mCost_traj_from_father);
	return steered;

}

void SystemState_Factory::Traj_to_target(Node* start, Node* trg, float* result) {

	this->Distance(start, trg, result);

	Node::Node_data* start_data = this->Get_data(start);
	VectorF* xo = &start_data->mPose;
	float* transient_time = &start_data->mState_additional.back()[0];
	VectorF* xf = &this->Get_data(trg)->mPose;

	VectorF A_lin; A_lin.Resize(this->pDyn_Syst->state_size()*this->pDyn_Syst->state_size());
	this->pDyn_Syst->eval_der_f(&A_lin, *xo);
	VectorF state_attual, u_attual, dx;
	VectorF dx_attual; dx_attual.Resize(this->pDyn_Syst->state_size());
	VectorF f_const; f_const.Resize(this->pDyn_Syst->state_size());

	this->pDyn_Syst->eval_f(&f_const, *xo);
	f_const *= -1.f;
	VectorF err_temp; err_temp.copy(*xo); err_temp -= *xf;
	addition_Matrix_x_Vector(&f_const, A_lin, err_temp);

	state_attual.copy(*xo);
	float integration_time = this->mDt;
	while (integration_time > *transient_time) {
		compute_u(&u_attual, f_const, start_data->mState_additional.front(), state_attual, *xf);
		compute_next_state(&state_attual, dx_attual, u_attual);

		if (!this->pDyn_Syst->is_admitted(state_attual, u_attual)) {
			*result = FLT_MAX;
			return;
		}

		if (state_attual.L2_distance(xf) <= this->mToll)
			break;

		integration_time += this->mDt;
	}

}
