/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "Problem_Navigation.h"
#include <string>
#include <sstream>

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
				std::vector<std::vector<float>> interp = handler->Compute_interpolated_path(*MT_RTT::json_parser::get_field(wayps_temp, "temp"));
				to_reprint += ",\"Solution\":" + MT_RTT::json_parser::load_JSON(interp) + "\n";
			}
		}
		else to_reprint += temp + "\n";
	}

	result = move(to_reprint);

}