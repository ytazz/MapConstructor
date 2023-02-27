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

struct mapPairRef {
	int mapFrom;
	int mapTo;
	real_t angleRef;
};

class MapDirectionOptimizer : public GaussNewton {
public:
	vector<mapPairRef> mprs;

	virtual bool Init();
	virtual bool CalcError();
#ifdef USE_JACOBIFUNC
	virtual bool CalcJacobi();
#endif
	virtual bool Update();
	virtual bool TermCond();
	virtual void PrintParams();

	MapDirectionOptimizer(const int param, const int error, const int method);
};

using namespace Scenebuilder;

class MapDirectionPreprocess: public TaskBase {
	int n;
public:
	MapDirectionPreprocess(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), n(0) {};
	virtual int Task(int argc, const char* argv[]);
};

}