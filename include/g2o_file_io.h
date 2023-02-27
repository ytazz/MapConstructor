#pragma once

#include "base.h"

#include <iostream>
#include <iomanip>
#include <string>

#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

namespace MapConstructor {

class G2OFileIO: public TaskBase {
public:
	string fileName;

	G2OFileIO(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), fileName() {};
	virtual int Task(int argc, const char* argv[]);
};

}