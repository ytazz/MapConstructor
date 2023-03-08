#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>
#include "converter.h"

#include "sbtypes.h"
#include "sbrollpitchyaw.h"
#include "sbconverter.h"
#include "sbxml.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

int ToG2OConverter::Task(int argc, const char* argv[]) {
	if (argc < 2 || string(argv[1]) == "Forward") {
		optimizer->clear();

		for (Map* map : *maps) {
			cout << "mapID : " << map->id << endl;
			string tag;
			XMLNode* NodeSetting;
			try {
				for (int i = 0;; i++) {
					NodeSetting = setting->GetNode("Node", i);
					int ID = -1;
					NodeSetting->Get<int>(ID, ".ID");
					if (ID == -1 || ID == map->id)
						break;
				}
			}
			catch (...) { continue; }
			NodeSetting->Get(tag, ".Tag");
			if (tag == "VERTEX_SE2")
				for (Node* node : map->nodes) {
					node->vertex = new VertexSE2;
					SE2 pose(node->location.pose.Pos().X(), node->location.pose.Pos().Y(), node->location.pose.Ori().Rotation().Z());
					node->vertex->setId(node->index);
					static_cast<VertexSE2*>(node->vertex)->setEstimate(pose);
					optimizer->addVertex(node->vertex);
					if (vertexId <= node->index)
						vertexId = node->index + 1;
				}
			else if (tag == "VERTEX_PROX")
				for (Node* node : map->nodes) {
					node->vertex = new VertexProx;
					SE2 pose(node->location.pose.Pos().X(), node->location.pose.Pos().Y(), node->location.pose.Ori().Rotation().Z());
					std::vector<Vector2> pp;
					for (int i = 0; i < node->proximities.size(); i++) {
						node->proximities[i]->index = i;
						pp.push_back(Vector2(node->proximities[i]->pos.X(), node->proximities[i]->pos.Y()));
					}
					node->vertex->setId(node->index);
					static_cast<VertexProx*>(node->vertex)->setEstimate(pose);
					static_cast<VertexProx*>(node->vertex)->setProximityParam(pp);
					optimizer->addVertex(node->vertex);
					if (vertexId <= node->index)
						vertexId = node->index + 1;
				}

			XMLNode* OdomSetting;
			try {
				for (int i = 0;; i++) {
					OdomSetting = setting->GetNode("Odom");
					int ID = -1;
					OdomSetting->Get<int>(ID, ".ID");
					if (ID == -1 || ID == map->id)
						break;
				}
			}
			catch (...) { continue; }
			OdomSetting->Get(tag, ".Tag");
			if (tag == "EDGE_SE2") {
				mat3_t info = mat3_t::Zero();
				vec3_t prInfoGain;
				OdomSetting->Get<vec3_t>(prInfoGain, ".poseRef-InfoGain");
				for (int i = 0; i < 3; i++)
					prInfoGain[i] = min(prInfoGain[i], 1e+8);
				vec3_t _mInfoGain;
				OdomSetting->Get<vec3_t>(_mInfoGain, ".movement-InfoGain");
				for (int i = 0; i < 3; i++)
					_mInfoGain[i] = min(_mInfoGain[i], 1e+8);
				mat3_t mInfoGain;
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						mInfoGain[i][j] = sqrt(_mInfoGain[i] * _mInfoGain[j]);
				vec3_t InfoLimit;
				OdomSetting->Get<vec3_t>(InfoLimit, ".infoLimit");
				for (int i = 0; i < 3; i++)
					InfoLimit[i] = min(max(InfoLimit[i], 1e-8), 1e+8);
				for (int i = 1; i < map->nodes.size(); i++) {
					Node* node = map->nodes[i];
					node->movement.odometry.first = new EdgeSE2;
					node->movement.odometry.second = nullptr;
					Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
					vec2_t pos;
					real_t angle(0.);
					if (node->movement.poseRef == pose_t(vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)) ||
						node->movement.prevNode == nullptr) {
						Node* prevNode = map->nodes[i - 1];
						pose_t poseRef3D = prevNode->location.pose.Inv() * node->location.pose;
						pos = vec2_t{ poseRef3D.Pos().X(), poseRef3D.Pos().Y() };
						angle = WrapPi(poseRef3D.Ori().Rotation().Z());
						info(0, 0) = pos.x != 0. ? min(prInfoGain[0] / (pos.x * pos.x), InfoLimit[0]) : InfoLimit[0];
						info(1, 1) = pos.y != 0. ? min(prInfoGain[1] / (pos.y * pos.y), InfoLimit[1]) : InfoLimit[1];
						info(2, 2) = angle != 0. ? min(prInfoGain[2] / (angle * angle), InfoLimit[2]) : InfoLimit[2];
						node->movement.odometry.first->vertices()[0] = optimizer->vertex(prevNode->vertex->id());
						node->movement.odometry.first->vertices()[1] = optimizer->vertex(node->vertex->id());
					}
					else {
						pos = vec2_t{ node->movement.poseRef.Pos().X(), node->movement.poseRef.Pos().Y() };
						angle = WrapPi(node->movement.poseRef.Ori().Rotation().Z());
						const array<int, 3> index = { 0, 1, 5 };
						for (int i = 0; i < 3; i++)
							for (int j = 0; j < 3; j++)
								info(i, j) = mInfoGain[i][j] * node->movement.info[index[i]][index[j]];
						node->movement.odometry.first->vertices()[0] = optimizer->vertex(node->movement.prevNode->vertex->id());
						node->movement.odometry.first->vertices()[1] = optimizer->vertex(node->vertex->id());
					}
					SE2 poseRef = SE2(pos.X(), pos.Y(), angle);
					static_cast<EdgeSE2*>(node->movement.odometry.first)->setMeasurement(poseRef);
					static_cast<EdgeSE2*>(node->movement.odometry.first)->setInformation(info);
					optimizer->addEdge(node->movement.odometry.first);
				}
			}
		}

		for (LoopMatch* lm : *matches) {
			cout << "matchID : " << lm->id << endl;
			string tag;
			XMLNode* LoopSetting;
			try {
				for (int i = 0;; i++) {
					LoopSetting = setting->GetNode("Loop", i);
					int ID = -1;
					LoopSetting->Get<int>(ID, ".ID");
					if (ID == -1 || ID == lm->id)
						break;
				}
			}
			catch (...) { continue; };
			LoopSetting->Get(tag, ".Tag");
			if (tag == "EDGE_SE2") {
				for (NodeMatch* nm : *lm) {
					nm->locMatch.loop.first = new EdgeSE2;
					nm->locMatch.loop.second = nullptr;
					pose_t poseRef3D = nm->locMatch.poseRef;
					vec2_t pos = vec2_t{ poseRef3D.Pos().X(), poseRef3D.Pos().Y() };
					real_t angle = WrapPi(poseRef3D.Ori().Rotation().Z());
					SE2 poseRef(pos.X(), pos.Y(), angle);
					Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
					const array<int, 3> index = { 0, 1, 5 };
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							info(i, j) = nm->locMatch.info[index[i]][index[j]];
					nm->locMatch.loop.first->vertices()[0] = optimizer->vertex(nm->f->vertex->id());
					nm->locMatch.loop.first->vertices()[1] = optimizer->vertex(nm->t->vertex->id());
					static_cast<EdgeSE2*>(nm->locMatch.loop.first)->setMeasurement(poseRef);
					static_cast<EdgeSE2*>(nm->locMatch.loop.first)->setInformation(info);
					optimizer->addEdge(nm->locMatch.loop.first);
				}
			}
			else if (tag == "EDGE_SWITCH_SE2") {
				real_t SwitchInfo;
				LoopSetting->Get<real_t>(SwitchInfo, ".switchInfo");
				for (NodeMatch* nm : *lm) {
					nm->locMatch.loop.first = new EdgeSwitchSE2;
					nm->locMatch.loop.second = new VertexSwitch;
					nm->locMatch.loop.second->setId(vertexId++);
					static_cast<VertexSwitch*>(nm->locMatch.loop.second)->setEstimate(1.);
					optimizer->addVertex(nm->locMatch.loop.second);
					pose_t poseRef3D = nm->locMatch.poseRef;
					vec2_t pos = vec2_t{ poseRef3D.Pos().X(), poseRef3D.Pos().Y() };
					real_t angle = WrapPi(poseRef3D.Ori().Rotation().Z());
					SE2 poseRef(pos.X(), pos.Y(), angle);
					Eigen::Matrix4d info = Eigen::Matrix4d::Zero();
					const array<int, 3> index = { 0, 1, 5 };
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							info(i, j) = nm->locMatch.info[index[i]][index[j]];
					info(3, 3) = SwitchInfo;
					nm->locMatch.loop.first->vertices()[0] = optimizer->vertex(nm->f->vertex->id());
					nm->locMatch.loop.first->vertices()[1] = optimizer->vertex(nm->t->vertex->id());
					nm->locMatch.loop.first->vertices()[2] = optimizer->vertex(nm->locMatch.loop.second->id());
					static_cast<EdgeSwitchSE2*>(nm->locMatch.loop.first)->setMeasurement(poseRef);
					static_cast<EdgeSwitchSE2*>(nm->locMatch.loop.first)->setInformation(info);
					optimizer->addEdge(nm->locMatch.loop.first);
				}
			}
			else if (tag == "EDGE_PROX") {
				real_t ProxSimGain = 1.;
				LoopSetting->Get<real_t>(ProxSimGain, ".proxSim-InfoGain");
				for (NodeMatch* nm : *lm)
					for (ProxMatch* pm : nm->proxMatches) {
						Eigen::Matrix2d info = Eigen::Matrix2d::Zero();
						real_t sim = (pm->similarity[0] + pm->similarity[1]) * 0.5;
						info(0, 0) = ProxSimGain > 0 ? sim * ProxSimGain : -ProxSimGain;
						info(1, 1) = ProxSimGain > 0 ? sim * ProxSimGain : -ProxSimGain;
						pm->loop.first = new EdgeProx;
						pm->loop.second = nullptr;
						pm->loop.first->vertices()[0] = optimizer->vertex(nm->f->vertex->id());
						pm->loop.first->vertices()[1] = optimizer->vertex(nm->t->vertex->id());
						static_cast<EdgeProx*>(pm->loop.first)->setMeasurement({ pm->f->index, pm->t->index });
						static_cast<EdgeProx*>(pm->loop.first)->setInformation(info);
						optimizer->addEdge(pm->loop.first);
					}
			}
			else if (tag == "EDGE_SWITCH_PROX") {
				real_t ProxSimGain, SwitchInfo;
				LoopSetting->Get<real_t>(ProxSimGain, ".proxSim-InfoGain");
				LoopSetting->Get<real_t>(SwitchInfo, ".switchInfo");
				for (NodeMatch* nm : *lm)
					for (ProxMatch* pm : nm->proxMatches) {
						Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
						info(0, 0) = ProxSimGain > 0 ? pm->similarity[0] * ProxSimGain : -ProxSimGain;
						info(1, 1) = ProxSimGain > 0 ? pm->similarity[1] * ProxSimGain : -ProxSimGain;
						info(2, 2) = SwitchInfo;
						pm->loop.first = new EdgeSwitchProx;
						pm->loop.second = new VertexSwitch;
						pm->loop.second->setId(vertexId++);
						static_cast<VertexSwitch*>(pm->loop.second)->setEstimate(1.);
						optimizer->addVertex(pm->loop.second);
						pm->loop.first->vertices()[0] = optimizer->vertex(nm->f->vertex->id());
						pm->loop.first->vertices()[1] = optimizer->vertex(nm->t->vertex->id());
						pm->loop.first->vertices()[2] = optimizer->vertex(pm->loop.second->id());
						static_cast<EdgeSwitchProx*>(pm->loop.first)->setMeasurement({ pm->f->index, pm->t->index });
						static_cast<EdgeSwitchProx*>(pm->loop.first)->setInformation(info);
						optimizer->addEdge(pm->loop.first);
					}
			}
			else if (tag == "EDGE_M_Est_PROX") {
				string strRobustKernel;
				real_t ProxSimGain, Delta;
				LoopSetting->Get<real_t>(ProxSimGain, ".proxSim-InfoGain");
				LoopSetting->Get<string>(strRobustKernel, ".robustKernel");
				LoopSetting->Get<real_t>(Delta, ".robustKernelDelta");
				for (NodeMatch* nm : *lm)
					for (ProxMatch* pm : nm->proxMatches) {
						g2o::RobustKernel* robustKernel;
						try { robustKernel = RobustKernelFactory::instance()->creator(strRobustKernel)->construct(); }
						catch (...) { return -1; }
						robustKernel->setDelta(Delta);
						Eigen::Matrix2d info = Eigen::Matrix2d::Zero();
						info(0, 0) = ProxSimGain > 0 ? pm->similarity[0] * ProxSimGain : -ProxSimGain;
						info(1, 1) = ProxSimGain > 0 ? pm->similarity[1] * ProxSimGain : -ProxSimGain;
						pm->loop.first = new EdgeMEstProx;
						pm->loop.second = nullptr;
						pm->loop.first->vertices()[0] = optimizer->vertex(nm->f->vertex->id());
						pm->loop.first->vertices()[1] = optimizer->vertex(nm->t->vertex->id());
						static_cast<EdgeMEstProx*>(pm->loop.first)->setMeasurement({ pm->f->index, pm->t->index });
						static_cast<EdgeMEstProx*>(pm->loop.first)->setInformation(info);
						static_cast<EdgeMEstProx*>(pm->loop.first)->setRobustKernel(robustKernel);
						static_cast<EdgeMEstProx*>(pm->loop.first)->setStrRobustKernel(strRobustKernel);
						optimizer->addEdge(pm->loop.first);
					}
			}
		}
	}

	if (string(argv[1]) == "Reverse") {
		for (Map* map : *maps)
			for (Node* node : map->nodes) {
				SE2 pose2D(static_cast<VertexSE2*>(node->vertex)->estimate());
				vec3_t angle(node->location.pose.Ori().Rotation().X(), node->location.pose.Ori().Rotation().Y(), WrapPi(pose2D.rotation().angle()));
				node->location.pose.Pos() = vec3_t{ pose2D.translation()[0], pose2D.translation()[1], node->location.pose.Pos().Z() };
				node->location.pose.Ori() = FromRollPitchYaw(angle);

				if (node->movement.odometry.first != nullptr) {
					node->movement.prevNode = map->nodes.FindByIndex(node->movement.odometry.first->vertices()[0]->id());
					//node.movement.node = map.nodes.FindByIndex(node.movement.odometry.first->vertices()[1]->id());
					vec2_t pos = vec2_t(static_cast<EdgeSE2*>(node->movement.odometry.first)->measurement().translation().x(),
						static_cast<EdgeSE2*>(node->movement.odometry.first)->measurement().translation().y());
					real_t angle = WrapPi(static_cast<EdgeSE2*>(node->movement.odometry.first)->measurement().rotation().angle());
					node->movement.poseRef.Pos() = vec3_t(pos.X(), pos.Y(), 0.);
					node->movement.poseRef.Ori() = FromRollPitchYaw(vec3_t(0., 0., angle));
					node->movement.info.clear();
					const array<int, 3> index = { 0, 1, 5 };
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
							node->movement.info[index[i]][index[j]]
							= static_cast<EdgeSE2*>(node->movement.odometry.first)->information()(i, j);
				}
			}
	}

	return -1;
}

}