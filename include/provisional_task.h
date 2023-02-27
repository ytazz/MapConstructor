#pragma once
#include "base.h"

#include <string.h>
#include <random>

#include "sbxml.h"
#include "sbtypes.h"
#define XML_STATIC

using namespace Scenebuilder;

namespace MapConstructor {

class Provisional : public TaskBase {

public:
	Provisional(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer) {};
	virtual int Task(int argc, const char* argv[]) {
		if (argc < 2)
			return false;
		if (string(argv[1]) == "") {
			//////////////
			return true;
		}
		return false;
	};
};

}