#pragma once
#include <iostream>
#include <boost/timer.hpp>

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

class Loc{
public:
	pose_t pose;
	Node* node;
	Loc(Node* _node) : pose(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)), node(_node) {}
	Loc() : Loc(nullptr) {}
};

class Prox {
public:
	vec3_t pos;
	int index;
	int id;
	int duration;
	real_t velocity;
	vec3_t spherical;
	Node* node;
	Prox(Node* _node) : pos(NAN, NAN, NAN), index(-1), id(-1), duration(-1), velocity(NAN), spherical(NAN, NAN, NAN), node(_node) {};
	Prox() : Prox(nullptr) {};
	void sph_to_xyz();
	void xyz_to_shp();
};

class Proxs : public vector<Prox> {
public:
	Node* node;
	Proxs(Node* _node) : vector<Prox>(), node(_node) {}
	Proxs() : Proxs(nullptr) {}
	Proxs(const Proxs& proxs);
	Proxs(const Proxs& proxs, Node* _node);
	Prox* FindById(const int _id);
	Prox* FindByIndex(const int _index);
};

class Geo {
public:
	int geoCount;
	int utcTime;
	int numSat;
	int quality;
	vec3_t llh;
	vec3_t ecef;
	vec3_t enu;
	vec3_t xyz;
	Node* node;
	Geo(Node* _node) : geoCount(-1), utcTime(-1), numSat(-1), quality(-1),
		llh(NAN, NAN, NAN), ecef(NAN, NAN, NAN), enu(NAN, NAN, NAN), xyz(NAN, NAN, NAN), node(_node) {}
	Geo() : Geo(nullptr) {}
	void llh_to_ecef();
	void ecef_to_enu(const Geo* origGeo);
	void enu_to_xyz(const real_t& AngOffset, const vec2_t& PosOffset);
};

class Point {
public:
	int id;
	vec3_t pos;
};

class PointCloud : public vector<Point> {
public:
	Point* FindById(const int _id);
	vector<Point*> FindNearest(const vec3_t _pos, int top = 1);
};

class PC_Loader {
public:
	PointCloud* pcMemory;
	string fileName;
	string delim;
	streampos row;
	Node* node;
	PC_Loader(Node* _node) : pcMemory(nullptr), fileName(), delim(","), row(ios_base::beg), node(_node) {}
	PC_Loader() : PC_Loader(nullptr) {}
	bool Load(const int& skip = 1, const vec2_t& verticalRange = {-FLT_MAX, FLT_MAX });
	bool Release() { delete pcMemory; return true; };
};

class Movement {
public:
	pose_t poseRef;
	mat6_t info;
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> odometry;
	Node* node;
	Node* prevNode;
	Movement(Node* _node) : poseRef(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)),
		info(), odometry(nullptr, nullptr), node(_node), prevNode(nullptr){}
	Movement() : Movement(nullptr) {}
};

class Node {
public:
	int index;
	int count;
	int time;
	Loc location;
	Proxs proximities;
	Geo geography;
//	PointCloud pc;
	PC_Loader pc;
	Movement movement;
	OptimizableGraph::Vertex* vertex;
	Map* map;
	Node(Map* _map) : index(-1), count(-1), time(-1),
		location(this), proximities(this), geography(this), movement(this),
		vertex(nullptr), map(_map) {}
	Node() : Node(nullptr) {}
	Node(const Node& node) : index(node.index), count(node.count), time(node.time),
		location(node.location), proximities(node.proximities), geography(node.geography), movement(node.movement), pc(node.pc),
		vertex(node.vertex), map(node.map) {
		location.node = this;
		proximities.node = this;
		geography.node = this;
		movement.node = this;
		pc.node = this;
	}
};

class Nodes : public vector<Node> {
public:
	Map* map;
	Nodes(Map* _map) : vector<Node>(), map(_map) {}
	Nodes() : Nodes(nullptr) {}
	Nodes(const Nodes& nodes);
	Nodes(const Nodes& nodes, Map* _map);
	Node* FindByIndex(const int _index);
	Node* FindByCount(const int _count);
	Node* FindByTime(const int _time);
};

class Map {
public:
	int id;
	Nodes nodes;
	Map() : id(-1), nodes(this) {}
	Map(const Map& _map) : id(_map.id), nodes(_map.nodes, this) {}
};

class Maps : public vector<Map> {
public:
	Maps() : vector<Map>() {}
	Map* FindById(const int _id);
};

class LocMatch {
public:
	pose_t poseRef;
	mat6_t info;
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> loop;
	LocMatch() : poseRef(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)), info(), loop(nullptr, nullptr) {}
};

class ProxMatch {
public:
	Prox* f;
	Prox* t;
	real_t similarity[2];
	pair<OptimizableGraph::Edge*, OptimizableGraph::Vertex*> loop;
	ProxMatch(Prox* _f, Prox* _t) : f(_f), t(_t), similarity(), loop(nullptr, nullptr) {}
	ProxMatch(const ProxMatch& pm);
};

class ProxMatches : public vector<ProxMatch> {
public:
	ProxMatches() : vector<ProxMatch>() {}
	ProxMatch* FindByProxs(const Prox* _proxf, const Prox* _proxt);
};

class NodeMatch {
public:
	Node* f;
	Node* t;
	LocMatch locMatch;
	ProxMatches proxMatches;
	NodeMatch(Node* _f, Node* _t) : f(_f), t(_t), locMatch(), proxMatches() {}
	bool ProxMatcher();
};

class LoopMatch : public vector<NodeMatch> {
public:
	int id;
	real_t score;
	bool reverse;
	LoopMatch() : vector<NodeMatch>(), id(-1), score(NAN), reverse() {}
	NodeMatch* FindByNodes(const Node* _nodef, const Node* _nodet);
};

class Matches : public vector<LoopMatch> {
public:
	Matches() : vector<LoopMatch>() {};
	LoopMatch* FindById(const int _id);
	NodeMatch* FindByNodes(const Node* _nodef, const Node* _nodet);
};

class TaskBase {
public:
	Scenebuilder::XMLNode* setting;
	Maps* maps;
	Matches* matches;
	SparseOptimizer* optimizer;
	TaskBase(Scenebuilder::XMLNode* _setting, Maps* _maps, Matches* _matches, SparseOptimizer* _optimizer)
		: setting(_setting), maps(_maps), matches(_matches), optimizer(_optimizer){};
	virtual int Task(int argc, const char* argv[]);
};

}