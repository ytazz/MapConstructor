#include "map_optimizer.h"

#include <iostream>
#include <iomanip>
#include <time.h>
#include <conio.h>

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

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	auto linearSolver = g2o::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
		g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
	SparseOptimizer optimizer;

	//cerr << "type ::  " << typeid(optimizer).name() << endl;
	optimizer.clear();

	optimizer.setAlgorithm(solver);

	optimizer.load(load_file.c_str());

	//VertexProx* firstRobotPose = dynamic_cast<VertexProx*>(optimizer.vertex(0));
	optimizer.vertex(0)->setFixed(true);
	optimizer.setVerbose(true);

	cerr << "Optimizing" << endl;
	optimizer.initializeOptimization();
	optimizer.optimize(n);
	cerr << "done." << endl;

	optimizer.save(save_file.c_str());

	optimizer.clear();

	return 0;
}

}