#include "base.h"
#include "map_loader.h"
#include "map_saver.h"
#include "g2o_file_io.h"
#include "g2o_optimizer.h"
#include "prox_optimizer.h"
#include "scan_matcher.h"
#include "converter.h"
#include "map_direction_optimizer.h"
#include "provisional_task.h"
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

MapConstructor::Maps maps;
MapConstructor::Matches matches;
Scenebuilder::XML setting;
g2o::SparseOptimizer optimizer;

// #define TaskOverride(str) new MapConstructor::##str(&maps, &matches)

int sequencer(Scenebuilder::XMLNode* TASKS) {
	vector<int> ID;
	setting.GetChildren(setting.GetRoot(), ID);
	MapConstructor::TaskBase* task = new MapConstructor::TaskBase(setting.GetNode(0), &maps, &matches, &optimizer);
	for (int i = 0;; i++) {
		string name, _name;
		Scenebuilder::XMLNode* TASK;
		try {
			if (TASKS->GetNode("task", i, false) == nullptr) break;
			TASK = TASKS->GetNode("task", i, false);
			name = MapConstructor::split(TASK->GetName(), ' ')[0];
		}
		catch (...) { break; };
		if (name == "for") {
			int n = 0;
			TASK->Get<int>(n, ".n");
			for (int i = 0; i < n; i++)
				sequencer(TASK);
			continue;
		}
		for (int j = 0; j < ID.size(); j++) {
			_name = setting.GetNode(ID[j])->GetName();
			if (name == _name) {
				const string task_line = TASK->GetName();
				vector<string> task_str(MapConstructor::split(task_line, ' '));
				vector<const char*> task_ch;
				for (string& str : task_str)
					task_ch.push_back(str.c_str());
				const int argc(task_str.size());
				const char** argv = new (const char*);
				std::copy(task_ch.begin(), task_ch.end(), argv);
				const string task_name = setting.GetNode(ID[j])->name;
				try {
					if (task_name == "MapOptimizer")
						task = new MapConstructor::MapOptimizer(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "ProxMatcher")
						task = new MapConstructor::ProxOptimizer(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "ScanMatcher")
						task = new MapConstructor::ScanMatching(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "MapDirPrepro")
						task = new MapConstructor::MapDirectionPreprocess(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "MapLoader")
						task = new MapConstructor::MapLoader(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "MapSaver")
						task = new MapConstructor::MapSaver(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "G2OFileIO")
						task = new MapConstructor::G2OFileIO(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "ToG2OConverter")
						task = new MapConstructor::ToG2OConverter(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "ToMapConverter")
						task = new MapConstructor::ToMapConverter(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else if (task_name == "Provisional")
						task = new MapConstructor::Provisional(setting.GetNode(ID[j]), &maps, &matches, &optimizer);
					else
						task = new MapConstructor::TaskBase(setting.GetNode(ID[j]), &maps, &matches, &optimizer);

					cout << endl << endl << "TASK" << i << " : " << task_line << endl;
					task->Task(argc, argv);
				}
				catch (...) { return -1; };
			}
		}
	}
	return 0;
}

int main(int argc, char** argv)
{

	//std::ofstream ofstr("../data/cerr.txt");
	//std::streambuf* coutbuf;
	//std::streambuf* cerrbuf;
	//coutbuf = std::cout.rdbuf(ofstr.rdbuf());
	//cerrbuf = std::cerr.rdbuf(ofstr.rdbuf());

	try {
		setting.Load("../conf/settings.xml");
	}
	catch (...) { return -1; };

	sequencer(setting.GetRootNode()->GetNode("SEQUENCE", false));

	cout << endl << endl << "Sequence completed." << endl
		<< "Hit any key." << endl;
	while (!_kbhit()); _getch();

	//std::cout.rdbuf(coutbuf);
	//std::cerr.rdbuf(cerrbuf);

	return 0;
}