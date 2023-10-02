#pragma once

#include "base.h"

#include <iostream>
#include <iomanip>
#include <string>

#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

namespace MapConstructor {

class MapSaver : public TaskBase {
public:
	ofstream saveFile;
	int minSatNum;
	int minQuality;
	vec3_t OrigLLH;
	real_t AngOffset;
	vec2_t PosOffset;

	virtual bool LocSaver(const int mapID);
	virtual bool MovementSaver(const int mapID);
	virtual bool ProxSaver(const int mapID);
	virtual bool GeoSaver(const int mapID);
	virtual bool MatchSaver();
	virtual bool PoseRefSaver();
	virtual bool AbsProxSaver(const int mapID); // For gnuplot execution
	virtual bool LocGeoSaver(const int mapID);
	virtual bool LocMatchSaver();               // For gnuplot execution
	virtual bool AbsProxMatchSaver();           // For gnuplot execution
	virtual bool SwitchVarSaver();
	//virtual bool SwitchVarSortedSaver();
	MapSaver(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer),
		saveFile(), minSatNum(0), minQuality(0), OrigLLH(), AngOffset(0.), PosOffset() {};
	virtual int Task(int argc, const char* argv[]);
};

}