/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#include "../../MT_RRT/Header/Planner.h"
#include "../../MT_RRT/Header/Planner_MT.h"
#include "../../MT_RRT/Header/json.h"
#include "Solve.h"
#include "Stream_Socket.h"
#include <fstream>
#include <iostream>

class GUI_Server {
public:
	struct I_Responder {
		virtual std::string compute_response(const std::string& request_head, const std::string& request_body) = 0;
	protected:
		struct profile_info {
			float							det_coeff;
			float							reall_coeff;
			size_t							iterations;
			size_t							Trial;
			size_t							strategy;
			MT_RTT::Node::I_Node_factory*   problem;
			std::list<size_t>				Threads;
		};
		std::string profile(const profile_info& info, const std::vector<float>& Qo, const std::vector<float>& Qf);
	};

	GUI_Server(I_Responder& responder);
	void Serve_forever();
private:
// data
	Stream_to_Client	Connection;
	I_Responder&		Resp;
};

void replace_line(const std::string& file, const std::string& line_before_to_replace, const std::string& new_line_to_put, const std::string& destination_file) { //used for printing profile results in html
	std::list<std::string> lines;
	std::ifstream f(file);
	if (!f.is_open()) throw 0;
	while (true) {
		lines.emplace_back();
		std::getline(f, lines.back());
		if (lines.back().compare(line_before_to_replace) == 0) break;
	}
	lines.emplace_back();
	std::getline(f, lines.back());
	lines.back() = std::move(new_line_to_put);
	while (!f.eof()) {
		lines.emplace_back();
		std::getline(f, lines.back());
	}
	f.close();

	std::ofstream of(destination_file);
	if (!of.is_open()) throw 0;
	while (!lines.empty()) {
		of << lines.front() << std::endl;
		lines.pop_front();
	}
	of.close();
}
std::string GUI_Server::I_Responder::profile(const profile_info& info, const std::vector<float>& Qo, const std::vector<float>& Qf) {
	std::vector<std::vector<float>> Results;
	std::string response = "{";
	std::string name;

	auto solver = MT_RTT::I_Planner::Get_canonical(info.det_coeff, info.iterations, info.problem);
	std::cout << std::endl << "Serial " << ":" << std::endl;
	solver->Cumulate_solutions();
	Results.push_back(std::vector<float>());
	Results.back() = Solve_using_trials(solver.get(), info.Trial, Qo, Qf, info.strategy);
	response += "\"serial\":" + MT_RTT::json_parser::load_JSON(&Results[0][0], Results[0].size());

	response += ",\"Threads\":[";
	auto it_Th = info.Threads.begin();
	response += std::to_string(*it_Th);
	it_Th++;
	for (it_Th; it_Th != info.Threads.end(); it_Th++)  response += "," + std::to_string(*it_Th);
	response += "]";
	response += ",\"parall\":[";

	solver = MT_RTT::I_Planner::Get_query___parall(info.det_coeff, info.iterations, info.problem);
	name = "Query_thread_pool";
	std::cout << std::endl << name << ":" << std::endl;
	Results = Solve_using_trials_threads(dynamic_cast<MT_RTT::I_Planner_MT*>(solver.get()), info.Threads, info.Trial, Qo, Qf, info.strategy);
	response += "{\"label\":\"" + name + "\",\"values\":" + MT_RTT::json_parser::load_JSON(Results) + ",\"color\":\"red\"}";

	solver = MT_RTT::I_Planner::Get_shared__parall(info.det_coeff, info.iterations, info.problem);
	name = "Shared_tree";
	std::cout << std::endl << name << ":" << std::endl;
	Results = Solve_using_trials_threads(dynamic_cast<MT_RTT::I_Planner_MT*>(solver.get()), info.Threads, info.Trial, Qo, Qf, info.strategy);
	response += ",{\"label\":\"" + name + "\",\"values\":" + MT_RTT::json_parser::load_JSON(Results) + ",\"color\":\"green\"}";

	solver = MT_RTT::I_Planner::Get_copied__parall(info.det_coeff, info.iterations, info.problem, 0, info.reall_coeff);
	name = "Copied_trees";
	std::cout << std::endl << name << ":" << std::endl;
	Results = Solve_using_trials_threads(dynamic_cast<MT_RTT::I_Planner_MT*>(solver.get()), info.Threads, info.Trial, Qo, Qf, info.strategy);
	response += ",{\"label\":\"" + name + "\",\"values\":" + MT_RTT::json_parser::load_JSON(Results) + ",\"color\":\"blue\"}";

	solver = MT_RTT::I_Planner::Get_multi_ag_parall(info.det_coeff, info.iterations, info.problem, 0, info.reall_coeff);
	name = "Multi_agents";
	std::cout << std::endl << name << ":" << std::endl;
	Results = Solve_using_trials_threads(dynamic_cast<MT_RTT::I_Planner_MT*>(solver.get()), info.Threads, info.Trial, Qo, Qf, info.strategy);
	response += ",{\"label\":\"" + name + "\",\"values\":" + MT_RTT::json_parser::load_JSON(Results) + ",\"color\":\"brown\"}";

	response += "]};";
	std::cout << "Profiling ended\r ";

	replace_line("../../src_JS/profile.html", "let time_data =", response, "./front_JS/profile.html");

	response = "Done";
	return move(response);
}

GUI_Server::GUI_Server(I_Responder& responder) : Connection(430), Resp(responder) { this->Connection.InitConnection(); }

void GUI_Server::Serve_forever() {

	std::string  request_head, request_body, response;

	while (true) {
		this->Connection.Recv(&request_head);
		this->Connection.Recv(&request_body);
		std::cout << "-----------------------------------\n";
		std::cout << "-->header: " << request_head << std::endl;
		std::cout << "-->  body: " << request_body << std::endl;

		response = this->Resp.compute_response(request_head, request_body);

		std::cout << "-->response: " << response << std::endl << std::endl;
		Connection.Send(response);
	}

}