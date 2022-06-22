#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include "map_optimizer.h"

#include "sbconverter.h"
#include "sbcsv.h"
#include "sbxml.h"

using namespace std;

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace g2o;

namespace g2o {
	G2O_USE_TYPE_GROUP(slam2d);
}

int Optimize_Proximity_PoseGraph(Scenebuilder::XMLNode *setting) {
	int n;
	setting->Get<int>(n, ".LoopNum");
	string load_file, save_file;
	setting->Get<string>(load_file, ".LoadFile");
	setting->Get<string>(save_file, ".SaveFile");

	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	SparseOptimizer optimizer;
	auto linearSolver = g2o::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
	OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(
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