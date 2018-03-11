
#include <cassert>
#include <cmath>
#include <chrono>
#include <vector>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char **argv) {
	using namespace std;
	using namespace Eigen;

	//CoM -> L4 -> Rz Rx Ry -> L3 -> Ry -> L2 -> Ry Rx -> L1 -> Tip
	//             1  2  3           4           5  6

	constexpr float deg_to_rad = M_PI/180;
	constexpr short number_of_freepoint = 6,
			        number_of_linkcount = 4;
	const IOFormat fmt(4, 0, ", ", "\n", "[", "]");

	auto start_p = chrono::high_resolution_clock::now();

	vector<float> link_lenght;
	vector<float> degrees, radians;
	vector<Vector3f> links;
	vector<Vector3f> directanse_of_rotations;
	vector<Quaternionf> quaternions;
	vector<Quaternionf> quaternion_cross;
	vector<Vector3f> tip_position_elements;
	Vector3f tip_position;

	tip_position = Vector3f::Zero();

	degrees.push_back(0.0);
	degrees.push_back(0.0);
	degrees.push_back(0.0);
	degrees.push_back(0.0);
	degrees.push_back(0.0);
	degrees.push_back(0.0);
	assert(degrees.size() == number_of_freepoint);

	for(auto &deg : degrees) {
		radians.push_back(deg * deg_to_rad);
	}

	link_lenght.push_back(1.0);
	link_lenght.push_back(1.0);
	link_lenght.push_back(1.0);
	link_lenght.push_back(1.0);
	assert(link_lenght.size() == number_of_linkcount);

	for(auto &l : link_lenght) {
		links.push_back(Vector3f(0, 0, l));
	}

	directanse_of_rotations.push_back(Vector3f::UnitZ());
	directanse_of_rotations.push_back(Vector3f::UnitX());
	directanse_of_rotations.push_back(Vector3f::UnitY());
	directanse_of_rotations.push_back(Vector3f::UnitY());
	directanse_of_rotations.push_back(Vector3f::UnitY());
	directanse_of_rotations.push_back(Vector3f::UnitX());
	assert(directanse_of_rotations.size() == radians.size());

	for(auto &dor : directanse_of_rotations) {
		static auto rad = radians.begin();
		quaternions.push_back(Quaternionf(AngleAxisf(*rad, dor)));
		rad++;
	}

	quaternion_cross.push_back(quaternions.at(0) * quaternions.at(1) * quaternions.at(2));
	quaternion_cross.push_back(quaternions.at(3));
	quaternion_cross.push_back(quaternions.at(4) * quaternions.at(5));

	for(auto &ll : links) {
		static int i = 0;
		tip_position_elements.push_back(ll);
		for(auto qc = quaternion_cross.rbegin(), qc_end = quaternion_cross.rend() - i; qc < qc_end; qc++) {
			tip_position_elements.at(i) = *qc * tip_position_elements.at(i);
		}
		i++;
	}

	for(auto &tpe : tip_position_elements) {
		tip_position = tip_position + tpe;
	}

	auto end_p = chrono::high_resolution_clock::now();
	auto interval_time = chrono::duration<float, ratio<1, 1000>>(end_p - start_p).count();

	cout << endl;
	cout << "Loss Time : " << interval_time << " x10^{-3}[sec]" << endl;
	cout << endl;

	cout << "Degree" << endl;
	for(auto &deg : degrees) {
		cout << deg << ", ";
	}
	cout << endl;

	cout << "Tip Posistion Vector" << endl;
	cout << tip_position.format(fmt) << endl;

	cout << endl;

	return 0;
}

