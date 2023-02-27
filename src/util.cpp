#include "util.h"

namespace MapConstructor {

template<class T>
void _swap(T& A, T& B) {
	T& _A = A;
	A = B;
	B = _A;
	return;
}

inline real_t WrapPi(const real_t _angle) {
	real_t angle = _angle;
	while (angle > M_PI)
		angle -= 2. * M_PI;
	while (angle <= -M_PI)
		angle += 2. * M_PI;
	return angle;
}

vector<string> split(const string& str, const char delim) {
	vector<string> vec;
	string s;
	stringstream ss(str);
	while (getline(ss, s, delim))
		vec.push_back(s);
	return vec;
}

}