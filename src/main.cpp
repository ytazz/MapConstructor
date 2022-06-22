#include "map_optimizer.h"
#include "save_weight_histogram.h"
#include "sbcsv.h"
#include "sbxml.h"
#include <conio.h>
#include <time.h>
#include <iostream>
#include <string>
#define XML_STATIC

using namespace std;

int main(int argc, char** argv)
{
	//if (Generate_G2Ofile(argc, argv) == 0) {
	//	cout << "wait for loop_closing" << endl;
	//	cout << "if loop_closed then press any key to next process" << endl;
	//	while (!_kbhit()); _getch();
	//	if (Optimize_Proximity_PoseGraph(argc, argv) == 0) {
	//		cout << "loop_closing was succeeded" << endl;
	//		cout << "press any key to generate optimized location file" << endl;
	//		while (!_kbhit()); _getch();
	//		if (Generate_Locfile(argc, argv) == 0) {
	//			cout << "press any key to end" << endl;
	//			while (!_kbhit());
	//			return 0;
	//		}
	//		return -1;
	//	}
	//	return -1;
	//}
	//return -1;
	//Conv_Geo2Loc("data/nc2021_D_geo.csv", "data/nc2021_D_locgeo.csv", 58500, -123.0, vec3_t{ 34.660647, 135.446145, 5.084000 });
	//Generate_G2Ofile(argc, argv);

	Scenebuilder::XML setting;
	try {
		setting.Load("../conf/settings.xml");
	}
	catch (...) {};

	try {
		Optimize_Proximity_PoseGraph(setting.GetRootNode()->GetNode("OptProxPoseGraph", false));
	}
	catch (...) {};
	while (!_kbhit());

	try {
		Save_Weight_Histgram(setting.GetRootNode()->GetNode("SaveWeightDistribution", false));
	}
	catch (...) {};
	while (!_kbhit());

	return 0;
}