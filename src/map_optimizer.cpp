#include "map_optimizer.h"

#include <iostream>
#include <iomanip>
#include <time.h>

using namespace std;
using namespace g2o;

namespace g2o {
	G2O_USE_TYPE_GROUP(slam2d);
}

namespace MapConstructor {

int MapOptimizer::Task(Scenebuilder::XMLNode *setting) {
	setting->Get<int>(n, ".LoopNum");
	setting->Get<string>(load_file, ".LoadFile");
	setting->Get<string>(save_file, ".SaveFile");

	auto linearSolver = g2o::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
	solver = new OptimizationAlgorithmGaussNewton(
		g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

	optimizer.setAlgorithm(solver);

	optimizer.load(load_file.c_str());

	//VertexProx* firstRobotPose = dynamic_cast<VertexProx*>(optimizer.vertex(0));
	optimizer.vertex(0)->setFixed(true);
	optimizer.setVerbose(true);

	//ParameterPPWInfo* ppwi;
	//ppwi->setInfo(10.);
	//optimizer.addParameter(ppwi);

	cerr << "Optimizing" << endl;
	optimizer.initializeOptimization();
	optimizer.optimize(n);
	cerr << "done." << endl;

	optimizer.save(save_file.c_str());

	optimizer.clear();

	return 0;
}

}