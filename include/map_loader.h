#pragma once

#include "base.h"

#include <iostream>
#include <iomanip>
#include <string>

#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

namespace MapConstructor {

class MapLoader : public TaskBase {
public:
	CsvReader loadFile;
	int NodeId;

	virtual bool LocLoader(const int mapID);
	virtual bool MovementLoader(const int mapID);
	virtual bool ProxLoader(const int mapID);
	virtual bool GeoLoader(const int mapID);
	virtual bool PointCloudLoader(const string filename, const int mapID);
	virtual bool MatchLoader();
	virtual bool PoseRefLoader();
	MapLoader(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer),
		NodeId(0) {};
	virtual int Task(int argc, const char* argv[]);
};

}