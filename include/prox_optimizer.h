#pragma once
#include "base.h"
#include "gauss_newton.h"

#include "sbsolver.h"
#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

#include <Eigen/Eigen>

#define USE_JACOBIFUNC

using namespace Eigen;

namespace MapConstructor {

class ProximityMatcher : public GaussNewton {
public:
	Vector2d pos_reff;
	double ang_reff;
	vector<Vector2d, Eigen::aligned_allocator<Vector2d> > prox[2];
	vector<int> pp[2];
	vector<double> omega;
	int maxItr;

	virtual bool Init();
	virtual bool CalcError();
#ifdef USE_JACOBIFUNC
	virtual bool CalcJacobi();
#endif
	virtual bool Update();
	virtual bool TermCond();
	virtual void PrintParams();

	ProximityMatcher(const int ppNum, const int method);
};

class ProxDirMatcher : public GaussNewton {
public:
	vector<real_t> prox[2];
	vector<int> pp[2];
	vector<double> omega;
	int maxItr;

	virtual bool Init();
	virtual bool CalcError();
#ifdef USE_JACOBIFUNC
	virtual bool CalcJacobi();
#endif
	virtual bool TermCond();
	virtual void PrintParams();

	ProxDirMatcher(const int ppNum, const int method);
};

using namespace Scenebuilder;

class ProxOptimizer : public TaskBase {
	int n;
public:
	ProxOptimizer(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), n(0) {};
	virtual int Task(int argc, const char* argv[]);
};

}