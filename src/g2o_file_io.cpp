#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>
#include "g2o_file_io.h"

#include "sbtypes.h"
#include "sbrollpitchyaw.h"
#include "sbconverter.h"
#include "sbxml.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

int G2OFileIO::Task(int argc, const char* argv[]) {
	if (argc < 2)
		return -1;
	if (string(argv[1]) == "Load") {
		setting->Get<string>(fileName, ".LoadFileName");
		return optimizer->load(fileName.c_str());
	}
	if (string(argv[1]) == "Save") {
		setting->Get<string>(fileName, ".SaveFileName");
		return optimizer->save(fileName.c_str());
	}
	return -1;
}

}