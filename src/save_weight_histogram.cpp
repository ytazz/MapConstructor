#include "save_weight_histogram.h"

#include <iostream>
#include <iomanip>
#include <conio.h>
#include <sstream>
#include <time.h>

namespace MapConstructor {

int SaveWeightHistgram::Task(Scenebuilder::XMLNode* setting) {
	setting->Get<string>(ifname, ".LoadFile");
	setting->Get<string>(ofname, ".SaveFile");
	ifcsv.Read(ifname, " ");
	ofs.open(ofname);
	const int row = ifcsv.NumRow();
	for (int i = 0; i < row; i++) {
		string tag = ifcsv.Get<string>(i, 0);
		if (tag != "VERTEX_SWITCH_PROX_PAIR") continue;
		const int col = ifcsv.Get<int>(i, 2);
		for (int j = 0; j < col; j++) {
			double weight = ifcsv.Get<double>(i, 3 + j);
			ofs << weight << endl;
		}
	}
	ifcsv.Clear();
	ofs.close();

	return 0;
}

}