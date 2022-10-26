#include "base.h"
#include "map_optimizer.h"
#include "map_converter.h"
#include "save_weight_histogram.h"
#include "geo_converter.h"
#include "prox_matcher.h"
#include "map_converter.h"
#include "sbxml.h"
#include <conio.h>
#include <time.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <array>


#define XML_STATIC

using namespace std;

int main(int argc, char** argv)
{

	//std::ofstream ofstr("../data/cerr.txt");
	//std::streambuf* coutbuf;
	//std::streambuf* cerrbuf;
	//coutbuf = std::cout.rdbuf(ofstr.rdbuf());
	//cerrbuf = std::cerr.rdbuf(ofstr.rdbuf());

	Scenebuilder::XML setting;
	try {
		setting.Load("../conf/settings.xml");
	}
	catch (...) {};

	Scenebuilder::XMLNode* TASK = setting.GetRootNode()->GetNode("SEQUENCE", false);
	vector<int> ID;
	setting.GetChildren(setting.GetRoot(), ID);
	for (int i = 0;;i++) {
		string name, _name;
		try {
			if (TASK->GetNode("task", i, false) == nullptr) break;
			name = TASK->GetNode("task", i, false)->GetName();
		}
		catch (...) { break; };
		for (int j = 0; j < ID.size(); j++) {
			_name = setting.GetNode(ID[j])->GetName();
			if (name == _name) {
				const string task_name = setting.GetNode(ID[j])->name;
				MapConstructor::TaskBase* task = new MapConstructor::TaskBase();
				try {
					if (task_name == "ProxMatcher")
						task = new MapConstructor::ProxMatcher();
					else if (task_name == "SaveWeightHistgram")
						task = new MapConstructor:: SaveWeightHistgram();
					else if (task_name == "GeoConverter")
						task = new MapConstructor::GeoConverter();
					else if (task_name == "MapOptimizer")
						task = new MapConstructor::MapOptimizer();
					else if (task_name == "MapConverter")
						task = new MapConstructor::MapConverter();
					task->Task(setting.GetNode(ID[j]));
				}
				catch (...) {};
			}
		}
	}

	cout << "Sequence completed." << endl;
	cout << "Hit any key." << endl;
	while (!_kbhit()); _getch();

	//std::cout.rdbuf(coutbuf);
	//std::cerr.rdbuf(cerrbuf);

	return 0;
}