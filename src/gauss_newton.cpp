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

GaussNewton::GaussNewton(const int param, const int error) : Itr(0), Obj(DBL_MAX), PrevObj(DBL_MAX), clacSimEqMethod(0){
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
}

GaussNewton::GaussNewton(const int param, const int error, const int method) : GaussNewton(param, error){
	clacSimEqMethod = method;
}

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

void GaussNewton::CalcHb() {
	b = Jacobi.transpose() * Omega * Error;
	H = Jacobi.transpose() * Omega * Jacobi;
}

void GaussNewton::CalcObj() {
	PrevObj = Obj;
	Obj = Error.dot(Omega * Error);
}

bool GaussNewton::CalcDelta() {
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

bool GaussNewton::Update() {
	for(int i = 0; i < Param.size(); i++)
		if(!Fixed[i])
			Param(i) += Delta(i);
	return true;
}

int GaussNewton::Step() {
	if (!CalcError()) return -1;
	if (!CalcJacobi()) return -1;
	CalcHb();
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

bool GaussNewton::setFixed(const int i) {
	if (i < 0 || i >= Fixed.size())
		return false;
	Fixed[i] = true;
	return true;
}

bool GaussNewton::setFree(const int i) {
	if (i < 0 || i >= Fixed.size())
		return false;
	Fixed[i] = false;
	return true;
}

}