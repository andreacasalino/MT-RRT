#include "../../src/Problem_Navigation.h"
#include "../../../src/GUI_Server.h"
#include <iostream>
using namespace std;


struct Responder : public GUI_Server::I_Responder {
	virtual std::string compute_response(const std::string& request_head, const std::string& request_body);
private:
	Navigator parse_scene(const vector<json_parser::field>& scene_json, vector<float>* Qo, vector<float>* Qf);
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
		Navigator Problem = parse_scene(info, &Qo, &Qf);

		auto params = json_parser::get_field(info, "params");
		float det_coeff = (*params)[0][0];
		size_t Iterations = (size_t)(*params)[0][1];
		size_t Thread = (size_t)(*params)[0][2];

		auto solver = I_Planner::Get_multi_ag_parall(det_coeff, Iterations, &Problem, Thread, 0.05f);
		solver->Set_post_processer<MT_RTT::Brute_force_Simplifier>();
		solver->RRT_star(Array(&Qo[0], Qo.size()), Array(&Qf[0], Qf.size()));
		list<Array> Waypoints = solver->Get_solution();

		if (Waypoints.empty()) response = "null";
		else {
		//interpolate the path
			Problem.Interpolate(Waypoints);
			vector<vector<float>> raw;
			raw.reserve(Waypoints.size());
			for (auto it = Waypoints.begin(); it != Waypoints.end(); ++it) {
				raw.emplace_back();
				raw.back() = { (*it)[0], (*it)[1], (*it)[2] };
			}
			response = json_parser::load_JSON(raw);
		}
	}

	else if (request_head.compare("prof") == 0) {
		info = json_parser::parse_JSON(request_body);
		Navigator Problem = parse_scene(info, &Qo, &Qf);

		auto params = json_parser::get_field(info, "params");
		profile_info profile_data;
		profile_data.det_coeff = (*params)[0][0];
		profile_data.iterations = (size_t)(*params)[0][1];
		profile_data.strategy = (size_t)(*params)[0][2];
		profile_data.Trial = (size_t)(*params)[0][3];
		profile_data.reall_coeff = (*params)[0][4];
		profile_data.problem = &Problem;
		for (size_t k = 0; k < (*params)[1].size(); k++) profile_data.Threads.emplace_back((size_t)(*params)[1][k]);

		response = this->profile(profile_data, Qo, Qf);
	}

	return response;
}

Navigator Responder::parse_scene(const vector<json_parser::field>& scene_json, vector<float>* Qo, vector<float>* Qf) {
	auto pt_temp = json_parser::get_field(scene_json, "Q_curr");
	*Qo = { (*pt_temp)[0][0], (*pt_temp)[0][1], (*pt_temp)[0][2] };
	pt_temp = json_parser::get_field(scene_json, "Q_trgt");
	*Qf = { (*pt_temp)[0][0], (*pt_temp)[0][1], (*pt_temp)[0][2] };
	return Navigator(scene_json);
}
