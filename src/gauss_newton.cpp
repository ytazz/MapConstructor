#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>
#include "gauss_newton.h"

#include "sbtypes.h"
#include "sbconverter.h"
#include "sbrollpitchyaw.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

// Constructor  Set the number of dimensions of each member variable and a solving sparse linear-equations
GaussNewton::GaussNewton(const int param, const int error, const int method) : Itr(0), Obj(DBL_MAX), PrevObj(DBL_MAX), clacSimEqMethod(method) {
	Param.resize(param);
	Param.setZero();
	Omega.resize(error, error);
	Omega.setZero();
	Error.resize(error);
	Error.setZero();
	Jacobi.resize(error, param);
	Jacobi.setZero();
	b.resize(param);
	b.setZero();
	H.resize(param, param);
	H.setZero();
	Delta.resize(param);
	Delta.setZero();
	Fixed.resize(param);
	std::fill(Fixed.begin(), Fixed.end(), false);
	clacSimEqMethod = method;
}

// Initialization
bool GaussNewton::Init() {
	CalcError();
	CalcObj();
	Itr = 0;
	Obj = PrevObj = DBL_MAX;
	Jacobi.setZero();
	b.setZero();
	H.setZero();
	Delta.setZero();
	return true;
}

// Numerical differentiation of the error function to obtain the Jacobi matrix
bool GaussNewton::CalcJacobi() {
	VectorXd LError(Error.size()),
		RError(Error.size());
	const double Delta = 1E-6;
	const double Scale = 1 / (2. * Delta);
	for (int i = 0; i < Param.size(); i++) {
		VectorXd AddParam = VectorXd::Zero(Param.size());
		AddParam[i] = Delta;
		Param -= AddParam;
		CalcError();
		LError = Error;
		Param += 2. * AddParam;
		CalcError();
		RError = Error;
		Param -= AddParam;
		Jacobi.block(0, i, Error.size(), 1) = (RError - LError) * Scale;
	}
	CalcError();
	return true;
}

// Calculate the objective function
void GaussNewton::CalcObj() {
	PrevObj = Obj; // Stores the value of the previous objective function
	Obj = Error.dot(Omega * Error);
}

// Calculate update
bool GaussNewton::CalcDelta() {
	b = Jacobi.transpose() * Omega * Error;
	H = Jacobi.transpose() * Omega * Jacobi;
	try {
		switch (clacSimEqMethod) {
		case ClacSimEqMethod::PartialPivLU:
			Delta = H.partialPivLu().solve(-b);
			break;
		case ClacSimEqMethod::FullPivLU:
			Delta = H.fullPivLu().solve(-b);
			break;
		case ClacSimEqMethod::HouseholderQR:
			Delta = H.householderQr().solve(-b);
			break;
		case ClacSimEqMethod::ColPivHouseholderQR:
			Delta = H.colPivHouseholderQr().solve(-b);
			break;
		case ClacSimEqMethod::FullPivHouseholderQR:
			Delta = H.fullPivHouseholderQr().solve(-b);
			break;
		case ClacSimEqMethod::CompleteOrthogonalDecomposition:
			Delta = H.completeOrthogonalDecomposition().solve(-b);
			break;
		case ClacSimEqMethod::LLT:
			Delta = H.llt().solve(-b);
			break;
		case ClacSimEqMethod::LDLT:
			Delta = H.ldlt().solve(-b);
			break;
		case ClacSimEqMethod::Inverse:
			if (H.norm() == 0.)
				Delta.setZero();
			else
				Delta = H.inverse()*(-b);
			break;
		}
	}
	catch (...) { return false; }
	return true;
}

// Update design variables
bool GaussNewton::Update() {
	for(int i = 0; i < Param.size(); i++)
		if(!Fixed[i])
			Param(i) += Delta(i);
	return true;
}

int GaussNewton::Step() {
	if (!CalcError()) return -1;
	if (!CalcJacobi()) return -1;
	if (!CalcDelta()) return -1;
	if (!Update()) return -1;
	Itr++;
	if (TermCond()) return true;
	return false;
}

int GaussNewton::Loop() {
	int flag = 0;
	while (1) {
		PrintParams();
		flag = Step();
		if (flag) break;
	}
	PrintParams();
	return flag;
}

// Reset the number of dimensions of design variables
bool GaussNewton::resizeParam(const int param){
	Param.resize(param);
	Param.setZero();
	Omega.setZero();
	Error.setZero();
	Jacobi.resize(Jacobi.rows(), param);
	Jacobi.setZero();
	b.resize(param);
	b.setZero();
	H.resize(param, param);
	H.setZero();
	Delta.resize(param);
	Delta.setZero();
	Fixed.resize(param);
	std::fill(Fixed.begin(), Fixed.end(), false);
	return true;
}

// Reset the number of dimensions of error function
bool GaussNewton::resizeError(const int error) {
	Param.setZero();
	Omega.resize(error, error);
	Omega.setZero();
	Error.resize(error);
	Error.setZero();
	Jacobi.resize(error, Jacobi.cols());
	Jacobi.setZero();
	b.setZero();
	H.setZero();
	Delta.setZero();
	std::fill(Fixed.begin(), Fixed.end(), false);
	return true;
}

// Fix the i-th design variable
bool GaussNewton::setFixed(const int i) {
	if (i < 0 || i >= Fixed.size())
		return false;
	Fixed[i] = true;
	return true;
}

// Free the i-th design variable
bool GaussNewton::setFree(const int i) {
	if (i < 0 || i >= Fixed.size())
		return false;
	Fixed[i] = false;
	return true;
}

}