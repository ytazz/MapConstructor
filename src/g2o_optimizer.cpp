#include "g2o_optimizer.h"

#include <iostream>
#include <iomanip>
#include <time.h>
#include <conio.h>

using namespace std;
using namespace g2o;

namespace MapConstructor {

int MapOptimizer::Task(int argc, const char* argv[]) {
	// Determine which solver and optimization method to use (Eigen, GaussNewton)
	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
	auto linearSolver = g2o::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
	OptimizationAlgorithmGaussNewton* solver
		= new OptimizationAlgorithmGaussNewton(g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));
	optimizer->setAlgorithm(solver);

	// Determine number of iterations
	int n = 0;
	setting->Get<int>(n, ".ItrNum");

	// Fix the first node
	optimizer->vertex(0)->setFixed(true);
	optimizer->setVerbose(true);

	cerr << "Optimizing" << endl;
	optimizer->initializeOptimization(); // Initial settings for nodes and edges
	optimizer->optimize(n); // Optimize nodes
	cerr << "done." << endl;

	return 0;
}

}