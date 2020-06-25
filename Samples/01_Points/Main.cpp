/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#include "src/Problem_points.h"
#include "../src/Solve.h"
#include "../src/Log.h"
using namespace std;
using namespace MT_RTT;

#define USE_MULTIPLE_STEER

//this kind of problem is described in Section 2.1 of the documentation
int main() {

	size_t Iterations = 2500;

//initialize a random scene with some random obstacles
	Problem_points Scene(5, 30, Point_2D(0.f, 0.f), Point_2D(1.f, 1.f));
#ifdef USE_MULTIPLE_STEER
	Scene.Set_Steer_iterations(6);
#endif

	
//define the starting and ending state to connect
	vector<float> S = { 0.f, 0.f };
	vector<float> E = { 1.f, 1.f };

	auto Scene_json = Scene.Get_as_JSON();

//check the behaviour of this function to understand how to use the planning algorithms
	auto Log_results = Solve_using_planners_and_strategies(Iterations, 0.1f, &Scene, S, E);
	
	Log_creator("../src_JS/Result_template.html", "Results/Serial.html", Scene_json, Log_results[0]);
	Log_creator("../src_JS/Result_template.html", "Results/Parallel_query.html", Scene_json, Log_results[1]);
	Log_creator("../src_JS/Result_template.html", "Results/Parallel_shared.html", Scene_json, Log_results[2]);
	Log_creator("../src_JS/Result_template.html", "Results/Parallel_copied.html", Scene_json, Log_results[3]);
	Log_creator("../src_JS/Result_template.html", "Results/Parallel_multiag.html", Scene_json, Log_results[4]);
// you can use your favorite browser to open the .html file created in the Results folder

	cout << "done\n";
	return 0;
}