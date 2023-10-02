#pragma once
#include <iostream>
#include <boost/timer/timer.hpp>

#include "sbxml.h"
#define XML_STATIC

#include "util.h"

#include "g2o/core/factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/robust_kernel_factory.h"

using namespace Scenebuilder;
using namespace g2o;

namespace g2o {
	G2O_USE_TYPE_GROUP(slam2d);
	G2O_USE_ROBUST_KERNEL(RobustKernelHuber);
	G2O_USE_ROBUST_KERNEL(RobustKernelCauchy);
	G2O_USE_ROBUST_KERNEL(RobustKernelGemanMcClure);
}

namespace MapConstructor {

class Loc;
class Prox;
class Proxs;
class Geo;
class Point;
class PointCloud;
class Movement;
class Node;
class Nodes;
class Map;
class Maps;
class ProxMatch;
class ProxMatches;
class NodeMatch;
class LoopMatch;
class Matches;

// < Robot pose
class Loc{
public:
	pose_t pose; // < Robot pose [m, quat]
	Node* node; // < parent

	Loc(Node* _node) : pose(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)), node(_node) {}
	Loc() : Loc(nullptr) {}
};

// < Proximity point position
class Prox : public UTRefCount {
public:
	vec3_t pos; // < Proximity point position [m]
	int index; // < Proximity point index (The x-th observed proximity point)
	int id; // < Proximity point id (Proximity point of the x-th object)
	int duration; // < Number of frames observed continuously
	real_t velocity; // < Velocity of the object to which the proximity point belongs [m/s]
	vec3_t spherical; // < Polar coordinates position (Horizontal angle, Vertical angle, Distance) [rad, m]
	Node* node; // < parent

	Prox(Node* _node) : pos(NAN, NAN, NAN), index(-1), id(-1), duration(-1), velocity(NAN), spherical(NAN, NAN, NAN), node(_node) {};
	Prox() : Prox(nullptr) {};

	// < Convert polar coordinate system to Euclidean coordinate system
	void sph_to_xyz();
	// < Convert Euclidean coordinate system to polar coordinate system
	void xyz_to_shp();
};

// < Proximity points
class Proxs : public vector<UTRef<Prox>> {
public:
	Node* node; // < parent

	Proxs(Node* _node) : vector<UTRef<Prox>>(), node(_node) {}
	Proxs() : Proxs(nullptr) {}

	// < Find proximity points by Prox::id
	Prox* FindById(const int _id);
	// < Find proximity points by Prox::index
	Prox* FindByIndex(const int _index); 
};

// < RTK Location Information
class Geo {
public:
	int geoCount; // < Count in Geo-file
	int utcTime; // < UTC time when RTK positioning
	int numSat; // < Number of satellites observed
	int quality; // < Positioning accuracy
	             // < 0:Impossible to positioning
	             // < 1:Single-positioning
	             // < 2:DGNSS
	             // < 4:RTK-GNSS fixed integers (Highest precision)
	             // < 5:RTK-GNSS floating integers
	vec3_t llh; // < LLH coordinates position [rad, m]
	vec3_t ecef; // < ECEF coordinates position [m]
	vec3_t enu; // < ENU coordinates position [m]
	vec3_t xyz; // < Correction ENU coordinates position [m]
	Node* node; // < parent

	Geo(Node* _node) : geoCount(-1), utcTime(-1), numSat(-1), quality(-1),
		llh(NAN, NAN, NAN), ecef(NAN, NAN, NAN), enu(NAN, NAN, NAN), xyz(NAN, NAN, NAN), node(_node) {}
	Geo() : Geo(nullptr) {}

	// < Convert LLH coordinate system to ECEF coordinate system
	void llh_to_ecef();
	// < Convert ECEF coordinate system to ENU coordinate system (origGeo : origin position)
	void ecef_to_enu(const Geo& origGeo);
	// < Correct ENU coordinate
	void enu_to_xyz(const real_t& AngOffset, const vec2_t& PosOffset);
};

// < LiDAR laser position
class Point {
public:
	int id; // < Point index (The x-th registered point)
	vec3_t pos; // < point position [m]
};

// < PointCloud
class PointCloud : public vector<Point> {
public:
	// < Find point by Point::id
	Point* FindById(const int _id);
	// < Find points near _pos
	vector<Point*> FindNearest(const vec3_t _pos, int top = 1);
};

// < Load and discard pointcloud (Reduced memory usage)
class PC_Loader {
public:
	PointCloud* pcMemory; // Store loaded pointcloud
	string fileName; // PointCloud-file's name
	string delim; // < delimited
	streampos row; // < Load position
	Node* node; // < parent

	PC_Loader(Node* _node) : pcMemory(nullptr), fileName(), delim(","), row(ios_base::beg), node(_node) {}
	PC_Loader() : PC_Loader(nullptr) {}

	// < Load pointcloud data (After PointCloud use, must do PC_Loader::Release(), **Note that it takes much times to load. Needs improvement
	bool Load(const int& countSkip = 1, const real_t& distSkip = FLT_MAX, const vec2_t& heightRange = {-FLT_MAX, FLT_MAX });
	// < Release stored pointcloud data (Reduced memory usage)
	bool Release() { delete pcMemory; return true; };
};

// < Movement of the robot in 1 frame
class Movement {
public:
	pose_t poseRef; // < Relative pose [m, quat]
	mat6_t info; // < Information matrix (Inverse of the covariance matrix)
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> odometry; // < Odometry edge (and Switch variables) (Used for g2o)
	Node* node; // < parent (current node)
	Node* prevNode; // < previous node

	Movement(Node* _node) : poseRef(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)),
		info(), odometry(nullptr, nullptr), node(_node), prevNode(nullptr){}
	Movement() : Movement(nullptr) {}
};

// < Collection of information available in single frame
class Node : public UTRefCount {
public:
	int index; // < The x-th loaded node
	int count; // < Number of frames since the start of the observation
	int time; // < Time from the start of the observation [ms]
	Loc location; // < Robot pose
	Proxs proximities; // < Proximity points
	Geo geography; // < RTK Location Information
//	PointCloud pc;
	PC_Loader pc; // < PointCloud Loader
	Movement movement; // < Movement of the robot in 1 frame
	OptimizableGraph::Vertex* vertex; // < vertex (Used for g2o)
	Map* map; // < parent

	Node(Map* _map) : index(-1), count(-1), time(-1),
		location(this), proximities(this), geography(this), movement(this), pc(this), 
		vertex(nullptr), map(_map) {}
	Node() : Node(nullptr) {}
};

// < Time series data
class Nodes : public vector<UTRef<Node>> {
public:
	Map* map; // < parent

	Nodes(Map* _map) : vector<UTRef<Node>>(), map(_map) {}
	Nodes() : Nodes(nullptr) {}
	Nodes(const Nodes& nodes);

	// < Find node by Node::index
	Node* FindByIndex(const int _index);
	// < Find node by Node::count
	Node* FindByCount(const int _count);
	// < Find node by Node::time
	Node* FindByTime(const int _time);
};

// < Map data obtained in 1 travel
class Map : public UTRefCount {
public:
	int id; // < Map id (set by loop-detection)
	Nodes nodes; // < Time series data

	Map() : id(-1), nodes(this) {}
};

// < Set of loaded map data
class Maps : public vector<UTRef<Map>> {
public:
	Maps() : vector<UTRef<Map>>() {}
	Map* FindById(const int _id);
};

// < Loop constraints by relative pose
class LocMatch {
public:
	pose_t poseRef; // < Relative pose [m, quat]
	mat6_t info; // < Information matrix
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> loop; // < Loop edge (and Switch variables) (Used for g2o)

	LocMatch() : poseRef(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)), info(), loop(nullptr, nullptr) {}
};

// < Loop constraints by proximity point
class ProxMatch : public UTRefCount {
public:
	Prox* f; // < Proximity points defining the pair 1
	Prox* t; // < Proximity points defining the pair 2
	real_t similarity[2]; // < Similarity of proximity point pairs
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> loop; // < Loop edge (and Switch variables) (Used for g2o)

	ProxMatch(Prox* _f, Prox* _t) : f(_f), t(_t), similarity(), loop(nullptr, nullptr) {}
	ProxMatch(const ProxMatch& pm);
};

// < Loop constraints by proximity points
class ProxMatches : public vector<UTRef<ProxMatch>> {
public:
	ProxMatches() : vector<UTRef<ProxMatch>>() {}
	ProxMatch* FindByProxs(const Prox* _proxf, const Prox* _proxt);
};

// < node pair
class NodeMatch : public UTRefCount {
public:
	Node* f; // < Node defining the pair 1
	Node* t; // < Node defining the pair 2
	LocMatch locMatch; // < Loop constraints by relative pose
	ProxMatches proxMatches; // < Loop constraints by proximity point

	NodeMatch(Node* _f, Node* _t) : f(_f), t(_t), locMatch(), proxMatches() {}
};

// < Loop section
class LoopMatch : public UTRefCount, public vector<UTRef<NodeMatch>> {
public:
	int id; // < Loop section id
	real_t score; // < Loop section similarity score
	bool reverse; // < Was the direction of travel of the robot in the loop section reversed?

	LoopMatch() : vector<UTRef<NodeMatch>>(), id(-1), score(NAN), reverse() {}

	// < Find NodeMatch by node pair
	NodeMatch* FindByNodes(const Node* _nodef, const Node* _nodet);
};

// < Set of loaded loop section
class Matches : public UTRefCount, public vector<UTRef<LoopMatch>> {
public:
	Matches() : vector<UTRef<LoopMatch>>() {};

	// < Find LoopMatch by LoopMatch::id
	LoopMatch* FindById(const int _id);
	// < Find NodeMatch by node pair
	NodeMatch* FindByNodes(const Node* _nodef, const Node* _nodet);
};

// < Base class for all tasks
class TaskBase {
public:
	Scenebuilder::XMLNode* setting; // < Setting by configuration file
	Maps* maps; // < Set of loaded map data
	Matches* matches; // < Set of loaded loop section
	SparseOptimizer* optimizer; // < Pose-graph data (Used for g2o)

	TaskBase(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: setting(_setting), maps(_maps), matches(_matches), optimizer(_optimizer){};

	// < Task contents
	virtual int Task(int argc, const char* argv[]);
};

}