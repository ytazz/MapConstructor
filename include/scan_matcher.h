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

class ScanMatcher : public GaussNewton {
public:
	Vector2d pos_reff;
	double ang_reff;
	PointCloud* pc[2];
	real_t ignoreRatio;
	vector<vector<Vector2d>> nps2D;
	int ignoreNum;
	int maxItr;

	virtual bool Init();
	virtual bool CalcError();
#ifdef USE_JACOBIFUNC
	virtual bool CalcJacobi();
#endif
	virtual bool Update();
	virtual bool TermCond();
	virtual void PrintParams();

	ScanMatcher(const int pointNum, const int method, const real_t _ignoreRatio);
};

using namespace Scenebuilder;

class ScanMatching : public TaskBase {
	int n;
public:
	ScanMatching(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), n(0) {};
	virtual int Task(int argc, const char* argv[]);
};

}