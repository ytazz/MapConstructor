#pragma once
#include <iostream>
#include <vector>
#include <string>

#include "sbtypes.h"

#include <Eigen/Eigen>

#define FIXEDINT(s, i) fixed << setw(i) << s << defaultfloat
#define FIXEDFLT(s, i, j) fixed << setw(i) << setprecision(j) << s << defaultfloat

using namespace Scenebuilder;

namespace MapConstructor {

typedef PTM::TMatrixCol<6, 6, real_t> mat6_t;

const double D2R = M_PI / 180.;
const double R2D = 180. / M_PI;

const double alpha = 6378137.;
const double e2 = 0.00669437999013;

template<class T>
static void _swap(T& A, T& B) {
	T _A = A;
	A = B;
	B = _A;
	return;
}

template<class T>
static void push_back(vector<T>& vec, const T& a) {
	vec.resize(vec.size() + 1, a);
	return;
}

template<class T>
static void push_back(vector<T>* vec, const T& a) {
	vec->resize(vec->size() + 1, a);
	return;
}

static inline real_t WrapPi(const real_t _angle) {
	real_t angle = _angle;
	while (angle > M_PI)
		angle -= 2. * M_PI;
	while (angle <= -M_PI)
		angle += 2. * M_PI;
	return angle;
}

static vector<string> split(const string& str, const char delim) {
	vector<string> vec;
	string s;
	stringstream ss(str);
	while (getline(ss, s, delim))
		vec.push_back(s);
	return vec;
}

static inline real_t cross2D(const Eigen::Vector2d a, const Eigen::Vector2d b) {
	return a.x() * b.y() - a.y() * b.x();
}

}