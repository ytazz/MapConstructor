#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>

#include "gauss_newton.h"
#include "map_direction_optimizer.h"

#include "sbtypes.h"
#include "sbconverter.h"
#include "sbrollpitchyaw.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {
// constructor  Sets the number of dimensions, etc. for the base class GaussNewton.
MapDirectionOptimizer::MapDirectionOptimizer(const int param, const int error, const int method) : GaussNewton(param, error, method) {};

// Initialization
bool MapDirectionOptimizer::Init() {
	// Reset the number of dimensions of the error function to the number of registered node pairs
	resizeError(mprs.size());
	Param.setZero();
	Omega.setIdentity();
	// Call Init() of base class GaussNewton
	return GaussNewton::Init();
}

// Calculate the relationship between the directions of two time series data
//  using the direction of travel between the node pair
bool MapDirectionOptimizer::CalcError() {
	int mapIdf, mapIdt;
	real_t mapRef;
	for (int row = 0; row < mprs.size() && row < Error.size(); row++) {
		mapIdf = mprs[row].mapFrom;
		mapIdt = mprs[row].mapTo;
		mapRef = mprs[row].angleRef;
		if (mapIdf == mapIdt)
			Error(row) = 0.;
		else
			Error(row) = WrapPi(Param(mapIdt) - Param(mapIdf) + mapRef);
	}
	return true;
}


#ifdef USE_JACOBIFUNC
// Calculate jacobi matrix without numerical differentiation
bool MapDirectionOptimizer::CalcJacobi() {
	int mapIdf, mapIdt;
	real_t mapRef;
	for (int row = 0; row < mprs.size() && row < Error.size(); row++) {
		mapIdf = mprs[row].mapFrom;
		mapIdt = mprs[row].mapTo;
		mapRef = mprs[row].angleRef;
		Jacobi.row(row).setZero();
		if (mapIdf == mapIdt)
			Jacobi.row(row).setZero();
		else {
			Jacobi(row, mapIdf) = -1.;
			Jacobi(row, mapIdt) = 1.;
		}
	}
	return true;
}
#endif

// Update design variables
bool MapDirectionOptimizer::Update() {
	for (int i = 0; i < Param.size(); i++)
		if (!Fixed[i])
			Param(i) = WrapPi(Param(i) + Delta(i));
	return true;
}

// Loop termination conditions
bool MapDirectionOptimizer::TermCond() {
	CalcError();
	CalcObj();
	if (fabs(Obj - PrevObj) < 1e-6 || Itr > 100) return true;
	return false;
}

// Display variables for each loop
inline void MapDirectionOptimizer::PrintParams() {
	stringstream ss;
	ss << "Itr:" << FIXEDINT(Itr, 3);
	ss << "\nParam: ";
	for (int i = 0; i < Param.size(); i++)
		ss << FIXEDFLT(Param(i), 6, 3) << " ";
	if (Obj != DBL_MAX)
		ss << "\nObj: " << FIXEDFLT(Obj, 6, 3) << " ";
	else
		ss << "\nObj: DBL_MAX ";
	//ss << "\nError: ";
	//for (int i = 0; i < Error.size(); i++)
	//	ss << FIXEDFLT(Error(i), 6, 3) << " ";
	//ss << "\nJacobi: ";
	//for (int i = 0; i < Jacobi.rows(); i++) {
	//	ss << "\n   ";
	//	for (int j = 0; j < Jacobi.cols(); j++)
	//		ss << FIXEDFLT(Jacobi(i, j), 6, 3) << " ";
	//}
	//ss << "\H b: ";
	//for (int i = 0; i < H.rows(); i++) {
	//	ss << "\n   ";
	//	for (int j = 0; j < H.cols(); j++)
	//		ss << FIXEDFLT(H(i, j), 6, 3) << " ";
	//	ss << FIXEDFLT(b(i), 6, 3) << " ";
	//}
	cout << ss.str() << endl;
}



int MapDirectionPreprocess::Task(int argc, const char* argv[]) {
	if (maps->size() <= 1)
		return -1;
	string clacSimEqMethodStr = "";
	setting->Get<string>(clacSimEqMethodStr, ".clacSimEqMethod");
	int clacSimEqMethod = ClacSimEqMethod::FindMethod(clacSimEqMethodStr);
	MapDirectionOptimizer mdo(maps->size(), 1, clacSimEqMethod);
	for(LoopMatch* lm : *matches)
		for (NodeMatch* nm : *lm) {
			// Register the time-series data ID and directional difference to which each node pair belongs
			mapPairRef mp;
			mp.mapFrom = nm->f->map->id - 1;
			mp.mapTo = nm->t->map->id - 1;
			mp.angleRef = WrapPi(nm->t->location.pose.Ori().Rotation().Z() - nm->f->location.pose.Ori().Rotation().Z() - (lm->reverse ? M_PI : 0.));
			mdo.mprs.push_back(mp);
		}
	mdo.Init();
	mdo.setFixed(0); // Fix the first time series data direction (Must always Fix one of the time series data direction)
	mdo.Loop();

	// Reflects optimization results in each time series data
	for (int id = 0; id < maps->size(); id++)
		for (Node* node : maps->at(id)->nodes)
			node->location.pose = pose_t::Rot(mdo.getParam()(id), 'z') * node->location.pose;

	return 0;
}

}