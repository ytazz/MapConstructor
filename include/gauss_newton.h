#pragma once
#include "base.h"

#include "sbsolver.h"
#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

#include <Eigen/Eigen>

using namespace Eigen;

namespace MapConstructor {

class ClacSimEqMethod {
public:
	enum {
		PartialPivLU,
		FullPivLU,
		HouseholderQR,
		ColPivHouseholderQR,
		FullPivHouseholderQR,
		CompleteOrthogonalDecomposition,
		LLT,
		LDLT,
		Inverse
	};
	ClacSimEqMethod(){};
	static int FindMethod(const string& str) {
		int Method = 0;
		if (str == "PartialPivLU")                         Method = ClacSimEqMethod::PartialPivLU;
		else if (str == "FullPivLU")                       Method = ClacSimEqMethod::FullPivLU;
		else if (str == "HouseholderQR")                   Method = ClacSimEqMethod::HouseholderQR;
		else if (str == "ColPivHouseholderQR")             Method = ClacSimEqMethod::ColPivHouseholderQR;
		else if (str == "FullPivHouseholderQR")            Method = ClacSimEqMethod::FullPivHouseholderQR;
		else if (str == "CompleteOrthogonalDecomposition") Method = ClacSimEqMethod::CompleteOrthogonalDecomposition;
		else if (str == "LLT")                             Method = ClacSimEqMethod::LLT;
		else if (str == "LDLT")                            Method = ClacSimEqMethod::LDLT;
		else if (str == "Inverse")                         Method = ClacSimEqMethod::Inverse;
		return Method;
	}
};

class GaussNewton{
protected:
	int Itr;
	VectorXd Param;
	MatrixXd Omega;
	VectorXd Error;
	MatrixXd Jacobi;
	VectorXd b;
	MatrixXd H;
	VectorXd Delta;
	double Obj, PrevObj;
	vector<bool> Fixed;
	int clacSimEqMethod;
public:
	virtual bool Init();
	virtual bool CalcError() { return 0; };
	virtual bool CalcJacobi();
	void CalcHb();
	void CalcObj();
	bool CalcDelta();
	virtual bool Update();
	virtual bool TermCond() { return 0; };
	virtual void PrintParams() {};
	virtual int Step();
	int Loop();

	bool resizeParam(const int param);
	bool resizeError(const int error);

	bool setFixed(const int i);
	bool setFree(const int i);

	inline int getItr() const { return Itr; };
	inline VectorXd getParam() const { return Param; };
	inline MatrixXd getOmega() const { return Omega; };
	inline VectorXd getError() const { return Error; };
	inline MatrixXd getJacobi() const { return Jacobi; };
	inline VectorXd getb() const { return b; };
	inline MatrixXd getH() const { return H; };
	inline double getObj() const { return Obj; };

	GaussNewton(const int param, const int error);
	GaussNewton(const int param, const int error, const int method);
};

}