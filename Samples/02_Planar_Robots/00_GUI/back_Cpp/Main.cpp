#include "../../src/Problem_planar_arms.h"
#include "../../../src/GUI_Server.h"
#include <float.h>
#include <math.h>
#include <iostream>
using namespace std;


struct Responder : public GUI_Server::I_Responder {
	virtual std::string compute_response(const std::string& request_head, const std::string& request_body);
private:
	unique_ptr<Node::I_Node_factory> parse_scene(const vector<json_parser::field>& scene_json, vector<float>* Qo, vector<float>* Qf);
	list<Array>				         interpolate(list<Array>& path);
// data
	vector<float>				Qo;
	vector<float>				Qf;
	vector<json_parser::field>	info;
};

int main() {

	cout << "Server C++\n";

	Responder resp;
	GUI_Server Server(resp);
	cout << "connection done\n";

	Server.Serve_forever();

	return 0;
}



std::string Responder::compute_response(const std::string& request_head, const std::string& request_body) {
	std::string response;
	if (request_head.compare("plan") == 0) {
		info = json_parser::parse_JSON(request_body);
		auto Problem = parse_scene(info, &Qo, &Qf);

		auto params = json_parser::get_field(info, "params");
		float det_coeff = (*params)[0][0];
		size_t mult_steer = (size_t)(*params)[0][1];
		size_t Iterations = (size_t)(*params)[0][2];
		size_t Thread = (size_t)(*params)[0][3];

		if (mult_steer > 1) Problem = unique_ptr<Node::I_Node_factory>(new Node_factory_multiple_steer(Problem, mult_steer));
		auto solver = I_Planner::Get_multi_ag_parall(det_coeff, Iterations, Problem.get(), Thread, 0.05f);
		solver->RRT_star(Array(&Qo[0], Qo.size()), Array(&Qf[0], Qf.size()));
		list<Array> Q_waypoints =  solver->Get_solution();

		if (Q_waypoints.empty()) response = "null";
		else {
			//interpolate the path
			auto Q_interp = interpolate(Q_waypoints);
			auto dofs = dynamic_cast<Scene_Proximity_calculator*>(dynamic_cast<Bubbles_free_configuration*>(Problem.get())->Get_proxier())->Get_Dofs();
			response = "[";
			auto it_w = Q_interp.begin();
			response += json_parser::load_JSON(&(*it_w)[0], dofs);
			it_w++;
			auto it_w_end = Q_interp.end();
			for (it_w; it_w != it_w_end; it_w++)
				response += "\n," + json_parser::load_JSON(&(*it_w)[0], dofs);
			response += "]";
		}
	}

	else if (request_head.compare("prof") == 0) {
		info = json_parser::parse_JSON(request_body);
		auto Problem = parse_scene(info, &Qo, &Qf);

		auto params = json_parser::get_field(info, "params");
		profile_info profile_data;
		profile_data.det_coeff = (*params)[0][0];
		profile_data.iterations = (size_t)(*params)[0][2];
		profile_data.strategy = (size_t)(*params)[0][3];
		profile_data.Trial = (size_t)(*params)[0][4];
		profile_data.reall_coeff = (*params)[0][5];
		profile_data.problem = Problem.get();
		for (size_t k = 0; k < (*params)[1].size(); k++) profile_data.Threads.emplace_back((size_t)(*params)[1][k]);

		size_t mult_steer = (size_t)(*params)[0][1];
		if (mult_steer > 1) Problem = unique_ptr<Node::I_Node_factory>(new Node_factory_multiple_steer(Problem, mult_steer));

		response = this->profile(profile_data, Qo, Qf);

	}

	else if (request_head.compare("ext_bubble") == 0) {
		info = json_parser::parse_JSON(request_body);
		auto Problem = parse_scene(info, &Qo, &Qf);

		float cost;
		bool trg_reached;
		vector<float> steered = Qo;
		Problem->Steer(&cost, &steered[0], &Qo[0], &Qf[0], &trg_reached);
		if (cost == FLT_MAX) response = "null";
		else				 response = json_parser::load_JSON(&steered[0], dynamic_cast<Scene_Proximity_calculator*>(dynamic_cast<Bubbles_free_configuration*>(Problem.get())->Get_proxier())->Get_Dofs());
	}

	else if (request_head.compare("check_coll") == 0) {
		info = json_parser::parse_JSON(request_body);
		Qo = import_pose(*json_parser::get_field(info, "Q_curr"));
		Scene_Collision_checker checker(info);

		if (checker.Collision_present(&Qo[0])) response = "1";
		else							       response = "0";
	}
	return response;
}

unique_ptr<Node::I_Node_factory> Responder::parse_scene(const vector<json_parser::field>& scene_json, vector<float>* Qo, vector<float>* Qf) {
	*Qo = import_pose(*json_parser::get_field(scene_json, "Q_curr"));
	*Qf = import_pose(*json_parser::get_field(scene_json, "Q_trgt"));
	unique_ptr<Bubbles_free_configuration::I_Proximity_calculator> scene(new Scene_Proximity_calculator(scene_json));
	unique_ptr<Node::I_Node_factory> hndl(new Bubbles_free_configuration(10.f, 4.712385f, -4.712385f, 
		dynamic_cast<Scene_Proximity_calculator*>(scene.get())->Get_Dof_tot(), scene));
	return hndl;
}

list<Array> Responder::interpolate(list<Array>& path) {
	list<Array> temp;
	auto it = path.begin();
	auto it_prev = it;
	auto it_end = path.end();
	temp.emplace_back(*it);
	it++;
	int N, N_att;
	float delta = 5.f * 3.141f / 180.f;
	size_t k, K = path.front().size();
	float* Delta = new float[K];
	float* Q_temp = new float[K];
	for (it; it != it_end; it++) {
		N = 1;
		for (k = 0; k < K; k++) {
			N_att = (int)ceilf(abs((*it)[k] - (*it_prev)[k]) / delta);
			if (N_att > N) N = N_att;
		}
		for (k = 0; k < K; k++) Delta[k] = ((*it)[k] - (*it_prev)[k]) / (float)N;

		for (int n = 1; n < N; n++) {
			for (k = 0; k < K; k++) {
				Q_temp[k] = temp.back()[k] + Delta[k];
				cout << " " << Q_temp[k];
			}
			cout << endl;
			temp.emplace_back(Array(&Q_temp[0], K));
		}
		temp.emplace_back(*it);

		it_prev++;
	}
	delete[] Delta;
	delete[] Q_temp;
	return temp;
}

