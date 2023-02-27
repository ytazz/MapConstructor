#pragma once

#include "base.h"

#include <iostream>
#include <iomanip>
#include <string>

#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

namespace MapConstructor {

class ToG2OConverter : public TaskBase {
public:
	int vertexId;
	ToG2OConverter(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), vertexId(0) {};
	virtual int Task(int argc, const char* argv[]);
};

class ToMapConverter : public TaskBase {
public:
	int vertexId;
	ToMapConverter(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer), vertexId(0) {};
	virtual int Task(int argc, const char* argv[]);
};

}