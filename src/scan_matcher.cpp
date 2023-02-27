#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>

#include "gauss_newton.h"
#include "scan_matcher.h"

#include "sbtypes.h"
#include "sbconverter.h"
#include "sbrollpitchyaw.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {
ScanMatcher::ScanMatcher(const int pointNum, const int method, const real_t _ignoreRatio) : GaussNewton(3, pointNum, method),
	pos_reff(0., 0.), ang_reff(0.), ignoreRatio(_ignoreRatio), nps2D(pointNum), ignoreNum(0), maxItr(20) {};

bool ScanMatcher::Init() {
	if (Param.size() == 3)
		Param << pos_reff[0], pos_reff[1], ang_reff;
	Omega.setIdentity();
	Omega /= real_t(Omega.rows());
	ignoreNum = int(ignoreRatio * pc[1]->size());
	return GaussNewton::Init();
}

bool ScanMatcher::CalcError() {
	const Vector2d Trs = Param.block<2, 1>(0, 0);
	const Rotation2Dd Rot(Param(2));
	Omega.setIdentity();
	Omega /= real_t(Omega.rows());
	real_t chi2 = 0.;
	for (int i = 0; i < nps2D.size() || i < pc[1]->size(); i++) {
		Vector2d pos2D(pc[1]->at(i).pos.X(), pc[1]->at(i).pos.Y());
		pos2D = Rot * pos2D + Trs;
		vector<Point*> np = pc[0]->FindNearest(vec3_t(pos2D.x(), pos2D.y(), pc[1]->at(i).pos.Z()), 2);
		for (Point* _np : np)
			nps2D[i].push_back(Vector2d(_np->pos.X(), _np->pos.Y()));
		Error(i, 0) = cross2D((nps2D[i][1] - nps2D[i][0]).normalized(), (nps2D[i][1] - pos2D));
		chi2 += Error(i, 0) * Error(i, 0);
	}
	chi2 /= pc[1]->size();
	for (int i = 0; i < nps2D.size() || i < pc[1]->size(); i++)
		if (Error(i, 0) * Error(i, 0) >= ignoreRatio * ignoreRatio * chi2)
			Omega(i, i) = 0.;
	return true;
}

#ifdef USE_JACOBIFUNC
bool ScanMatcher::CalcJacobi() {
	const Vector2d Trs = Param.block<2, 1>(0, 0);
	const Rotation2Dd Rot(Param(2));
	for (int i = 0; i < nps2D.size() || i < pc[1]->size(); i++) {
		Vector2d pos2D(pc[1]->at(i).pos.X(), pc[1]->at(i).pos.Y());
		pos2D = Rot * pos2D + Trs;
		Vector2d e = (nps2D[i][1] - nps2D[i][0]).normalized();
		Jacobi.block<1, 3>(i, 0) << -cross2D(e, Vector2d(1., 0.)), -cross2D(e, Vector2d(0., 1.)), -e.dot(pos2D);
	}
	return true;
}
#endif


bool ScanMatcher::Update() {
	Vector3d _Param = Param;
	const Rotation2Dd Rot(Delta(2));
	Param.block<2, 1>(0, 0) = Rot * _Param.block<2, 1>(0, 0) + Delta.block<2, 1>(0, 0);
	Param(2) = _Param(2) + Delta(2);
	return true;
}


bool ScanMatcher::TermCond() {
	CalcError();
	CalcObj();
	if (fabs(Obj - PrevObj) < 1e-6 || Itr >= maxItr) return true;
	return false;
}

inline void ScanMatcher::PrintParams() {
	//stringstream ss;
	//ss << "Itr:" << FIXEDINT(Itr, 3);
	//ss << endl << "Param ";
	//for (int i = 0; i < Param.size(); i++)
	//	ss << FIXEDFLT(Param(i), 6, 3) << " ";
	//ss << endl << "Error: ";
	//for (int i = 0; i < 10 && i < Error.size(); i++)
	//	ss << FIXEDFLT(Error(i), 6, 3) << " ";
	//if (Obj != DBL_MAX)
	//	ss << "   Obj: " << FIXEDFLT(Obj, 6, 3) << " ";
	//else
	//	ss << "   Obj: DBL_MAX ";
	//ss << endl << "Jacobi: ";
	//for (int i = 0; i < 5 && i < Jacobi.rows(); i++) {
	//	ss << endl << "   ";
	//	for (int j = 0; j < Jacobi.cols(); j++)
	//		ss << FIXEDFLT(Jacobi(i, j), 6, 3) << " ";
	//}
	////ss << "\nHessi: ";
	////for (int i = 0; i < Hessi.rows(); i++) {
	////	ss << "\n   ";
	////	for (int j = 0; j < Hessi.cols(); j++)
	////		ss << FIXEDFLT(Hessi(i, j), 6, 3) << " ";
	////}
	//ss << endl;
	//cout << ss.str();
}



int ScanMatching::Task(int argc, const char* argv[]) {
	string clacSimEqMethodStr = "";
	setting->Get<string>(clacSimEqMethodStr, ".clacSimEqMethod");
	int clacSimEqMethod = ClacSimEqMethod::FindMethod(clacSimEqMethodStr);
	int maxItr = 20;
	setting->Get<int>(maxItr, ".maxItr");
	vec2_t verticalRange = { -FLT_MAX, FLT_MAX };
	setting->Get<vec2_t>(verticalRange, ".verticalRange");
	int skip = 10;
	setting->Get<int>(skip, ".skip");
	real_t ignoreRatio = 0.;
	setting->Get<real_t>(ignoreRatio, ".ignoreRatio");

	for (LoopMatch& lm : *matches) {
		cout << "matchID : " << lm.id << endl;
		XMLNode* MatchSetting;
		try {
			for (int i = 0;; i++) {
				MatchSetting = setting->GetNode("Match", i);
				int ID = -1;
				MatchSetting->Get<int>(ID, ".ID");
				if (ID == -1 || ID == lm.id) 
					break;
			}
		}
		catch (...) { continue; }
		for (NodeMatch& nm : lm) {
			Vector2d pos_reff = Vector2d::Zero();
			real_t ang_reff = lm.reverse ? M_PI : 0.;
			//cout << "fnode : " << nm.f->map->id << "." << nm.f->count
			//	<< "     tnode : " << nm.t->map->id << "." << nm.t->count << endl;
			nm.locMatch.poseRef.clear();
			nm.locMatch.info.clear();
			nm.f->pc.Load(skip, verticalRange);
			nm.t->pc.Load(skip, verticalRange);
			if (nm.f->pc.pcMemory->size() < 10 || nm.t->pc.pcMemory->size() < 10) {
				nm.f->pc.Release();
				nm.t->pc.Release();
				continue;
			}
			ScanMatcher sm(nm.t->pc.pcMemory->size(), clacSimEqMethod, ignoreRatio);
			sm.pc[0] = nm.f->pc.pcMemory;
			sm.pc[1] = nm.t->pc.pcMemory;
			sm.pos_reff = pos_reff;
			sm.ang_reff = ang_reff;
			sm.maxItr = maxItr;
			sm.Init();
			if (sm.Loop() == 1) {
				pos_reff = sm.pos_reff = Vector2d(sm.getParam()(0), sm.getParam()(1));
				ang_reff = sm.ang_reff = sm.getParam()(2);
				Eigen::Matrix3d hessi = sm.getH();
				nm.locMatch.poseRef.Pos() = vec3_t(sm.pos_reff.x(), sm.pos_reff.y(), 0.);
				nm.locMatch.poseRef.Ori() = FromRollPitchYaw(vec3_t(0., 0., sm.ang_reff));
				const int index[3] = { 0, 1, 5 };
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						nm.locMatch.info[index[i]][index[j]] = hessi(i, j);
			}
			nm.f->pc.Release();
			nm.t->pc.Release();
		}
	}

	return 0;
}

}