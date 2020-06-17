#include "../src/Problem_Navigation.h"
#include "../../../MT_RRT/Header/json.h"
#include <fstream>
#include <iostream>
#include <ctime>
#include <float.h>
using namespace std;

float sample_U(const float& u_min, const float& u_max);

vector<vector<float>> log_traj(Planar_trajectory& trj);

int main() {

	srand(time(NULL));

	ofstream f("result.json");
	f << "[";

	float coords[4];
	for (size_t k = 0; k < 50; k++) {
		if (k > 0) f << ",";
		f << "{\"sol\":";
		for(size_t kk=0; kk<4; kk++) coords[kk] =  sample_U(-5.f, 5.f);	
		Planar_trajectory traj(1.f, coords[0] , coords[1], sample_U(-3.141f, 3.141f), coords[2], coords[3], sample_U(-3.141f, 3.141f));
		vector<vector<float>> pos;
		cout << "-------------------------------------------------\n";
		if (traj.get_cost() != FLT_MAX) pos = log_traj(traj);
		f << MT_RTT::json_parser::load_JSON(pos);
		f << ",\"cost\":" << to_string(traj.get_cost());
		f << "}";
	}


	f << "]";
	f.close();

	return 0;
}


float sample_U(const float& u_min, const float& u_max) {

	float delta = u_max - u_min;
	return  u_min + delta * (float)rand() / (float)RAND_MAX;

}

vector<vector<float>> log_traj(Planar_trajectory& trj) {

	Planar_trajectory::iterator it(&trj, 0.05f);
	vector<vector<float>> pos;
	while (it.is_not_at_end()) {
		pos.push_back({ it.get_position()[0] , it.get_position()[1] , it.get_position()[2] });
		++it;
	}
	return move(pos);

}