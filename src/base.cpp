#include <iostream>
#include <sstream>
#include <string.h>
#include <array>
#include "base.h"
#include "util.h"

#include "sbtypes.h"
#include "sbconverter.h"
#include "sbrollpitchyaw.h"
#include "sbcsv.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

void Prox::sph_to_xyz() {
	const real_t theta = spherical[0];
	const real_t phi = spherical[1];
	const real_t dist = spherical[2];
	pos.X() = dist * cos(phi) * cos(theta);
	pos.Y() = dist * cos(phi) * sin(theta);
	pos.Z() = dist * sin(phi);
}

void Prox::xyz_to_shp() {
	const real_t x = pos.X();
	const real_t y = pos.Y();
	const real_t z = pos.Z();
	if (x || y) spherical[0] = atan2(y, x);
	else        spherical[0] = 0.;
	if (x || y || z) spherical[1] = atan2(z, sqrt(x * x + y * y));
	else             spherical[1] = 0.;
	spherical[2] = sqrt(x * x + y * y + z * z);
}



Prox* Proxs::FindById(const int _id) {
	for (Prox* prox : *this)
		if (prox->id == _id)
			return prox;
	return 0;
}

Prox* Proxs::FindByIndex(const int _index) {
	for (Prox* prox : *this)
		if (prox->index == _index)
			return prox;
	return 0;
}



void Geo::llh_to_ecef() {
	const double Clat = cos(llh[0]), Slat = sin(llh[0]);
	const double chi = sqrt(1. - e2 * Slat * Slat);
	ecef[0] = (alpha / chi + llh[2]) * Clat * cos(llh[1]);
	ecef[1] = (alpha / chi + llh[2]) * Clat * sin(llh[1]);
	ecef[2] = (alpha * (1. - e2) / chi + llh[2]) * Slat;
}

void Geo::ecef_to_enu(const Geo& origGeo) {
	const double Clat = cos(origGeo.llh[0]), Slat = sin(origGeo.llh[0]),
		Clon = cos(origGeo.llh[1]), Slon = sin(origGeo.llh[1]);
	const mat3_t ConvMat
		= { -Slon,			Clon,			0.,
			-Slat * Clon,	-Slat * Slon,	Clat,
			Clat * Clon,	Clat * Slon,	Slat };
	enu = ConvMat * (ecef - origGeo.ecef);
}

void Geo::enu_to_xyz(const real_t& AngOffset, const vec2_t& PosOffset) {
	xyz = mat3_t::Rot(AngOffset, 'z') * enu + vec3_t(PosOffset.x, PosOffset.y, 0.);
}



Point* PointCloud::FindById(const int _id) {
	for (Point& point : *this)
		if (point.id == _id)
			return &point;
	return 0;
}

vector<Point*> PointCloud::FindNearest(const vec3_t _pos, int top) {
	if(top < 1)
		return vector<Point*>();
	vector<real_t> dists;
	vector<Point*> points;
	dists.resize(top);
	fill(dists.begin(), dists.end(), FLT_MAX);
	points.resize(top);
	for (Point& point : *this) {
		real_t dist = (point.pos - _pos).square();
		Point* point_ = &point;
		int i = 0;
		for (; i < top; i++)
			if (dist < dists[i]) {
				_swap<real_t>(dist, dists[i]);
				_swap<Point*>(point_, points[i]);
				break;
			}
		for (i++; i < top; i++) {
			_swap<real_t>(dist, dists[i]);
			_swap<Point*>(point_, points[i]);
		}
	}
	return points;
}



bool PC_Loader::Load(const int& countSkip, const real_t& distSkip, const vec2_t& heightRange) {
	ifstream pcFile(fileName);
	pcFile.seekg(row);
	pcMemory = new PointCloud;
	string str;
	if(!getline(pcFile, str))
		return false;
	string str2;
	stringstream ss(str);
	for(int i = 0; i <= 2; i++)
		getline(ss, str2, ',');
	streampos g = ss.tellg();
	int numPoint = atoi(str2.c_str());
	vector<pair<real_t, vec3_t>> pcSort;
	for (int i = 0;; i++) {
		bool flag = false;
		vector<string> pos_str(3);
		for (int j = 0; j <= 2; j++)
			if (!getline(ss, pos_str[j], ',')) {
				flag = true;
				break;
			}
		if (flag)
			break;
		vec3_t point = vec3_t{ atof(pos_str[0].c_str()), atof(pos_str[1].c_str()), atof(pos_str[2].c_str()) } * 0.001;
		//real_t verticalAngle = atan2(point[2] - 0.8, sqrt(point[0] * point[0] + point[1] * point[1]));
		if (heightRange[0] < point[2] && point[2] < heightRange[1])
			pcSort.push_back(pair<real_t, vec3_t>(atan2(point[1], point[0]), point));
		ss.seekg(21 * (countSkip - 1), ios_base::cur);
	}

	//std::sort(pcSort.begin(), pcSort.end());
	
	const real_t sqDistSkip = distSkip * distSkip;
	for (pair<real_t, vec3_t>& _pcSort : pcSort) {
		bool flag = false;
		for (int i = 0; i < pcMemory->size(); i++) {
			//vec2_t diff2D = vec2_t(_pcSort.second[0] - pcMemory->at(pcMemory->size() - i - 1).pos[0],
			//	                   _pcSort.second[1] - pcMemory->at(pcMemory->size() - i - 1).pos[1]);
			if ((_pcSort.second - pcMemory->at(pcMemory->size() - i - 1).pos).square() < sqDistSkip) {
				flag = true;
				break;
			}
		}
		if (flag)
			continue;
		Point point;
		point.pos = _pcSort.second;
		point.id = pcMemory->size();
		pcMemory->push_back(point);
	}

	return true;
}



Nodes::Nodes(const Nodes& nodes) : vector<UTRef<Node>>(nodes), map(nodes.map) {
	for (Node* node : *this)
		node->map = nodes.map;
};

Node* Nodes::FindByIndex(const int _index) {
	for (Node* node : *this)
		if (node->index == _index)
			return node;
	return 0;
}

Node* Nodes::FindByCount(const int _count) {
	for (Node* node : *this)
		if (node->count == _count)
			return node;
	return 0;
}

Node* Nodes::FindByTime(const int _time) {
	for (Node* node : *this)
		if (node->time == _time)
			return node;
	return 0;
}



Map* Maps::FindById(const int _id) {
	for (Map* map : *this)
		if (map->id == _id)
			return map;
	return 0;
}



ProxMatch::ProxMatch(const ProxMatch& pm) : f(pm.f), t(pm.t), similarity() {
	this->similarity[0] = pm.similarity[0];
	this->similarity[1] = pm.similarity[1];
}



ProxMatch* ProxMatches::FindByProxs(const Prox* _proxf, const Prox* _proxt) {
	for (ProxMatch* pm : *this)
		if ((pm->f == _proxf && pm->t == _proxt) ||
			(pm->f == _proxt && pm->t == _proxf))
			return pm;
	return 0;
}



NodeMatch* LoopMatch::FindByNodes(const Node* _nodef, const Node* _nodet) {
	for (NodeMatch* nm : *this)
		if ((nm->f == _nodef && nm->t == _nodet) ||
			(nm->f == _nodet && nm->t == _nodef))
			return nm;
	return 0;
}



LoopMatch* Matches::FindById(const int _id) {
	for (LoopMatch* lm : *this)
		if (lm->id == _id)
			return lm;
	return 0;
}

NodeMatch* Matches::FindByNodes(const Node* _nodef, const Node* _nodet) {
	for (LoopMatch* lm : *this)
		for (NodeMatch* nm : *lm)
			if ((nm->f == _nodef && nm->t == _nodet) ||
				(nm->f == _nodet && nm->t == _nodef))
				return nm;
	return 0;
}



int TaskBase::Task(int argc, const char* argv[]) {
	std::cout << "Did not match the task." << std::endl;
	return 0;
};

}