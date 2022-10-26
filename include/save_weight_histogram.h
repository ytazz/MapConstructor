#pragma once
#include "base.h"

#include <fstream>
#include <string.h>

#include "sbconverter.h"
#include "sbcsv.h"
#include "sbxml.h"
#define XML_STATIC

using namespace Scenebuilder;

namespace MapConstructor {

class SaveWeightHistgram : public TaskBase {
	string ifname, ofname;
	CsvReader ifcsv;
	ofstream ofs;
public:
	SaveWeightHistgram() :TaskBase() {};
	virtual int Task(Scenebuilder::XMLNode*);
};

}