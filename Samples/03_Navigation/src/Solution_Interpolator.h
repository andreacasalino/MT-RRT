/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "Problem_Navigation.h"
#include <string>
#include <sstream>

std::list<Array> Import(const std::vector<std::vector<float>>& waypoints) {

	std::list<Array> traj;
	for (size_t k = 0; k < waypoints.size(); ++k) traj.emplace_back(&waypoints[k][0], waypoints[k].size());
	return traj;

}

void Interp_solutions(std::string& result, Navigator* handler) {

	std::string to_reprint;
	std::string solution_raw, temp;
	std::stringstream fi(result);
	while (!fi.eof()) {
		std::getline(fi, temp);
		if (temp.compare(",\"Solution\":[") == 0) {
			to_reprint.pop_back();
			solution_raw = "\"temp\":[";
			while (true) {
				std::getline(fi, temp);
				if (temp.compare("]") == 0) break;
				solution_raw += temp;
			}
			solution_raw += "]";
			if (solution_raw.compare(",\"Solution\":[]") == 0) to_reprint += solution_raw + "\n";
			else {
			//interpolate the waypoints
				auto wayps_temp = MT_RTT::json_parser::parse_JSON(solution_raw);
				std::list<Array> waypoints = Import(*MT_RTT::json_parser::get_field(wayps_temp, "temp"));
				handler->Interpolate(waypoints);
				std::vector<std::vector<float>> to_print;
				to_print.reserve(waypoints.size());
				for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
					to_print.emplace_back();
					to_print.back() = {(*it)[0] , (*it)[1], (*it)[2] };
				}
				to_reprint += ",\"Solution\":" + MT_RTT::json_parser::load_JSON(to_print) + "\n";
			}
		}
		else to_reprint += temp + "\n";
	}

	result = to_reprint;

}