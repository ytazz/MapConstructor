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
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"

using namespace g2o;

namespace MapConstructor {

class MapOptimizer : public TaskBase {

public:
	MapOptimizer(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: TaskBase(_setting, _maps, _matches, _optimizer) {};
	virtual int Task(int argc, const char* argv[]);
};

}