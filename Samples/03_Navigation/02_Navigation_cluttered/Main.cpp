/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "../src/Problem_Navigation.h"
#include "../../src/Solve.h"
#include "../../src/Log.h"
#include "../src/Solution_Interpolator.h"
using namespace std;
using namespace MT_RTT;

vector<float> import_config(const vector<json_parser::field>& fields, const string& name);

//this kind of problem is described in Section 2.3 of the documentation
int main() {

	size_t Iterations = 5000;
	vector<float> Qo, Qf;

//read the problem from the textual file
	auto Scene_raw = get_content_of_file("problem.json");
	vector<json_parser::field> fields = json_parser::parse_JSON(Scene_raw);

	unique_ptr<Navigator> Scene(new Navigator(fields));

	Qo = import_config(fields, "Q_curr");
	Qf = import_config(fields, "Q_trgt");

//check the behaviour of this function to understand how to use the planning algorithms
	auto Log_results = Solve_using_planners_and_strategies(Iterations, 0.1f, Scene.get(), Qo, Qf);

//interpolate the solutions before logging the result
	Interp_solutions(Log_results[0], Scene.get());
	Log_creator("../../src_JS/Result_template.html", "Results/Serial.html", Scene_raw, Log_results[0]);
	Interp_solutions(Log_results[1], Scene.get());
	Log_creator("../../src_JS/Result_template.html", "Results/Parallel_query.html", Scene_raw, Log_results[1]);
	Interp_solutions(Log_results[2], Scene.get());
	Log_creator("../../src_JS/Result_template.html", "Results/Parallel_shared.html", Scene_raw, Log_results[2]);
	Interp_solutions(Log_results[3], Scene.get());
	Log_creator("../../src_JS/Result_template.html", "Results/Parallel_copied.html", Scene_raw, Log_results[3]);
	Interp_solutions(Log_results[4], Scene.get());
	Log_creator("../../src_JS/Result_template.html", "Results/Parallel_multiag.html", Scene_raw, Log_results[4]);
// you can use your favorite browser to open the .html file created in the Results folder

	cout << "done\n";
	return 0;
}

vector<float> import_config(const vector<json_parser::field>& fields, const string& name) {

	auto data = json_parser::get_field(fields, name);
	vector<float> Config;
	Config.reserve(3);
	Config.emplace_back((*data)[0][0]);
	Config.emplace_back((*data)[0][1]);
	Config.emplace_back((*data)[0][2]);
	return Config;

}
