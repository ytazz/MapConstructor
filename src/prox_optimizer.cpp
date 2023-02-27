#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>

#include "gauss_newton.h"
#include "prox_optimizer.h"

#include "sbtypes.h"
#include "sbconverter.h"
#include "sbrollpitchyaw.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {
ProximityMatcher::ProximityMatcher(const int ppNum, const int method) : GaussNewton(3, 2 * ppNum, method),
	pos_reff(0., 0.), ang_reff(0.), maxItr(20){
	pp[0].resize(ppNum);
	pp[1].resize(ppNum);
	omega.resize(ppNum);
};

bool ProximityMatcher::Init() {
	if (Param.size() == 3)
		Param << pos_reff[0], pos_reff[1], ang_reff;
	if (Omega.size() == 4 * omega.size() * omega.size())
		for (int i = 0; i < omega.size(); i++)
			Omega(2 * i, 2 * i) = Omega(2 * i + 1, 2 * i + 1) = omega[i];
	return GaussNewton::Init();
}

bool ProximityMatcher::CalcError() {
	const Vector2d Trs = Param.block(0, 0, 2, 1);
	const Rotation2Dd Rot(Param(2));
	for (int i = 0; i < pp[0].size(); i++) {
		const Vector2d ref_prox[2] = { prox[0][pp[0][i]], Rot * prox[1][pp[1][i]] + Trs };
		const Vector2d Dref_prox = ref_prox[1] - ref_prox[0];
		const Vector2d n[2] = { ref_prox[0].normalized(), (ref_prox[1] - Trs).normalized() };
		const double l[2] = { Dref_prox.dot(n[0]), -Dref_prox.dot(n[1]) };
		Error.block<2, 1>(2 * i, 0) << l[0], l[1];
	}
	return true;
}

#ifdef USE_JACOBIFUNC
bool ProximityMatcher::CalcJacobi() {
	const Vector2d Trs = Param.block(0, 0, 2, 1);
	const Rotation2Dd Rot(Param(2));
	const Rotation2Dd dRot(Param(2) + M_PI * 0.5);
	const Vector2d ref_prox[3];
	for (int i = 0; i < pp[0].size(); i++) {
		const Vector2d ref_prox[3] = { prox[0][pp[0][i]], Rot * prox[1][pp[1][i]] + Trs, dRot * prox[1][pp[1][i]] };
		const Vector2d n[3] = { ref_prox[0].normalized(), (ref_prox[1] - Trs).normalized(), ref_prox[2].normalized() };
		const double dl[2][3] = {
			{n[0].x(), n[0].y(), n[0].dot(ref_prox[2])},
			{-n[1].x(), -n[1].y(), n[2].dot(ref_prox[0] - Trs)}
		};
		Jacobi.block<2, 3>(2 * i, 0) << dl[0][0], dl[0][1], dl[0][2],
										dl[1][0], dl[1][1], dl[1][2];
	}
	return true;
}
#endif


bool ProximityMatcher::Update() {
	Vector3d _Param = Param;
	const Rotation2Dd Rot(Delta(2));
	Param.block<2, 1>(0, 0) = Rot * _Param.block<2, 1>(0, 0) + Delta.block<2, 1>(0, 0);
	Param(2) = _Param(2) + Delta(2);
	return true;
}


bool ProximityMatcher::TermCond() {
	CalcError();
	CalcObj();
	if (fabs(Obj - PrevObj) < 1e-6 || Itr >= maxItr) return true;
	return false;
}

inline void ProximityMatcher::PrintParams() {
	//stringstream ss;
	//ss << "Itr:" << FIXEDINT(Itr, 3) << " : Param ";
	//for (int i = 0; i < Param.size(); i++)
	//	ss << FIXEDFLT(Param(i), 6, 3) << " ";
	//ss << "\nError: ";
	//for (int i = 0; i < Error.size(); i++)
	//	ss << FIXEDFLT(Error(i), 6, 3) << " ";
	//if (Obj != DBL_MAX)
	//	ss << "   Obj: " << FIXEDFLT(Obj, 6, 3) << " ";
	//else
	//	ss << "   Obj: DBL_MAX ";
	////ss << "\nJacobi: ";
	////for (int i = 0; i < Jacobi.rows(); i++) {
	////	ss << "\n   ";
	////	for (int j = 0; j < Jacobi.cols(); j++)
	////		ss << FIXEDFLT(Jacobi(i, j), 6, 3) << " ";
	////}
	////ss << "\Hessi: ";
	////for (int i = 0; i < Hessi.rows(); i++) {
	////	ss << "\n   ";
	////	for (int j = 0; j < Hessi.cols(); j++)
	////		ss << FIXEDFLT(Hessi(i, j), 6, 3) << " ";
	////}
	//cout << ss.str() << endl;
}



ProxDirMatcher::ProxDirMatcher(const int ppNum, const int method) : GaussNewton(1, ppNum, method), maxItr(20){
	pp[0].resize(ppNum);
	pp[1].resize(ppNum);
	omega.resize(ppNum);
};

bool ProxDirMatcher::Init() {
	Param.setZero();
	Omega.setZero();
	if (Omega.size() == omega.size() * omega.size())
		for (int i = 0; i < omega.size(); i++)
			Omega(i, i) = omega[i];
	return GaussNewton::Init();
}

bool ProxDirMatcher::CalcError() {
	const Rotation2Dd Rot(Param(0));
	for (int i = 0; i < pp[0].size(); i++)
		Error(i, 0) = (Param(0) + prox[1][pp[1][i]]) - prox[0][pp[0][i]];
	return true;
}

#ifdef USE_JACOBIFUNC
bool ProxDirMatcher::CalcJacobi() {
	for (int i = 0; i < pp[0].size(); i++)
		Jacobi(i, 0) = 1.;
	return true;
}
#endif


bool ProxDirMatcher::TermCond() {
	CalcError();
	CalcObj();
	if (fabs(Obj - PrevObj) < 1e-6 || Itr >= maxItr) return true;
	return false;
}

inline void ProxDirMatcher::PrintParams() {
	//stringstream ss;
	//ss << "Itr:" << FIXEDINT(Itr, 3) << " : Param ";
	//for (int i = 0; i < Param.size(); i++)
	//	ss << FIXEDFLT(Param(i), 6, 3) << " ";
	//ss << "\nError: ";
	//for (int i = 0; i < Error.size(); i++)
	//	ss << FIXEDFLT(Error(i), 6, 3) << " ";
	//if (Obj != DBL_MAX)
	//	ss << "   Obj: " << FIXEDFLT(Obj, 6, 3) << " ";
	//else
	//	ss << "   Obj: DBL_MAX ";
	////ss << "\nJacobi: ";
	////for (int i = 0; i < Jacobi.rows(); i++) {
	////	ss << "\n   ";
	////	for (int j = 0; j < Jacobi.cols(); j++)
	////		ss << FIXEDFLT(Jacobi(i, j), 6, 3) << " ";
	////}
	////ss << "\Hessi: ";
	////for (int i = 0; i < Hessi.rows(); i++) {
	////	ss << "\n   ";
	////	for (int j = 0; j < Hessi.cols(); j++)
	////		ss << FIXEDFLT(Hessi(i, j), 6, 3) << " ";
	////}
	//cout << ss.str() << endl;
}



int ProxOptimizer::Task(int argc, const char* argv[]) {
	string clacSimEqMethodStr = "";
	setting->Get<string>(clacSimEqMethodStr, ".clacSimEqMethod");
	int clacSimEqMethod = ClacSimEqMethod::FindMethod(clacSimEqMethodStr);
	int maxItr = 20;
	setting->Get<int>(maxItr, ".maxItr");

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
			ProxDirMatcher pdm(nm.proxMatches.size(), clacSimEqMethod);
			ProximityMatcher pm(nm.proxMatches.size(), clacSimEqMethod);
			for (int i = 0; i < nm.proxMatches.size(); i++) {
				pm.pp[0][i] = pdm.pp[0][i] = nm.proxMatches[i].f->index;
				pm.pp[1][i] = pdm.pp[1][i] = nm.proxMatches[i].t->index;
				pm.omega[i] = pdm.omega[i] = 0.5 * (nm.proxMatches[i].similarity[0] + nm.proxMatches[i].similarity[1]);
			}

			nm.locMatch.poseRef.clear();
			nm.locMatch.info.clear();
			if (pm.getError().size() > 3) {
				pm.pos_reff = Vector2d::Zero();
				pm.ang_reff = 0;
				for (Prox& prox : nm.f->proximities) {
					pm.prox[0].push_back(Vector2d(prox.pos.X(), prox.pos.Y()));
					pdm.prox[0].push_back(atan2(prox.pos.Y(), prox.pos.X()));
				}
				for (Prox& prox : nm.t->proximities) {
					pm.prox[1].push_back(Vector2d(prox.pos.X(), prox.pos.Y()));
					pdm.prox[1].push_back(atan2(prox.pos.Y(), prox.pos.X()));
				}
				pdm.maxItr = maxItr;
				pdm.Init();
				if (pdm.Loop() == 1)
					pm.ang_reff = pdm.getParam()(0);
				pm.maxItr = maxItr;
				pm.Init();
				if (pm.Loop() == 1) {
					pm.pos_reff = pm.getParam().block<2, 1>(0, 0);
					pm.ang_reff = pm.getParam()(2);
					Eigen::Matrix3d hessi = pm.getH();
					nm.locMatch.poseRef.Pos() = vec3_t(pm.pos_reff.x(), pm.pos_reff.y(), 0.);
					nm.locMatch.poseRef.Ori() = FromRollPitchYaw(vec3_t(0., 0., pm.ang_reff));
					const int index[3] = { 0, 1, 5 };
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							nm.locMatch.info[index[i]][index[j]] = hessi(i, j);
				}
			}
		}
	}

	return 0;
}

}