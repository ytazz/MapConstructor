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