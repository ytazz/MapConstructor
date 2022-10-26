#pragma once
#include "base.h"

#include <string.h>

#include "sbxml.h"
#define XML_STATIC

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

namespace MapConstructor {

class MapOptimizer : public TaskBase {
	typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
	typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	int n;
	string load_file, save_file;
	OptimizationAlgorithmGaussNewton* solver;
	SparseOptimizer optimizer;

public:
	MapOptimizer() :TaskBase() {};
	virtual int Task(Scenebuilder::XMLNode*);
};

}