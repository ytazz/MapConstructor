#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>
#include "map_saver.h"

#include "sbtypes.h"
#include "sbrollpitchyaw.h"
#include "sbconverter.h"
#include "sbxml.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

bool MapSaver::LocSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	saveFile << "count, time, "
		<< "location_pos_x, location_pos_y, location_pos_z, "
		<< "location_roll, location_pitch, location_yaw, " << endl;
	for (Node& node : map->nodes) {
		vec3_t rpy = ToRollPitchYaw(node.location.pose.Ori());
		saveFile << node.count << ", "
			<< node.time << ", "
			<< FIXEDFLT(node.location.pose.Pos().X(), 6, 3) << ", "
			<< FIXEDFLT(node.location.pose.Pos().Y(), 6, 3) << ", "
			<< FIXEDFLT(node.location.pose.Pos().Z(), 6, 3) << ", "
			<< FIXEDFLT(rpy.X(), 6, 3) << ", "
			<< FIXEDFLT(rpy.Y(), 6, 3) << ", "
			<< FIXEDFLT(rpy.Z(), 6, 3) << ", " << endl;
	}
	return true;
}

bool MapSaver::MovementSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	saveFile << "count, time, "
		<< "movement_pos_x, movement_pos_y, movement_pos_z, "
		<< "movement_roll, movement_pitch, movement_yaw, info, " << endl;
	for (Node& node : map->nodes) {
		if (node.movement.prevNode == nullptr
			|| node.movement.poseRef == (vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)))
			continue;
		vec3_t rpy = ToRollPitchYaw(node.movement.poseRef.Ori());
		saveFile << node.count << ", "
			<< node.time << ", "
			<< FIXEDFLT(node.movement.poseRef.Pos().X(), 6, 3) << ", "
			<< FIXEDFLT(node.movement.poseRef.Pos().Y(), 6, 3) << ", "
			<< FIXEDFLT(node.movement.poseRef.Pos().Z(), 6, 3) << ", "
			<< FIXEDFLT(rpy.X(), 6, 3) << ", "
			<< FIXEDFLT(rpy.Y(), 6, 3) << ", "
			<< FIXEDFLT(rpy.Z(), 6, 3) << ", ";
		for (int i = 0; i < 6; i++)
			for (int j = i; j < 6; j++)
				saveFile << FIXEDFLT((node.movement.info[i][j] + node.movement.info[j][i]) * 0.5, 6, 3) << ", ";
		saveFile << endl;
	}
	return true;
}

bool MapSaver::ProxSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	saveFile << "count, time, nprox, "
		<< "proximity_id, proximity_duration, proximity_velocity, "
		<< "proximity_theta, proximity_phi, proximity_distance, " << endl;
	for (Node& node : map->nodes) {
		saveFile << node.count << ", "
			<< node.time << ", "
			<< node.proximities.size() << ", ";
		for (Prox& prox : node.proximities)
			saveFile << prox.id << ", "
				<< prox.duration << ", "
				<< FIXEDFLT(prox.velocity, 6, 3) << ", "
				<< FIXEDFLT(prox.spherical[0], 6, 3) << ", "
				<< FIXEDFLT(prox.spherical[1], 6, 3) << ", "
				<< FIXEDFLT(prox.spherical[2], 6, 3) << ", ";
		saveFile << endl;
	}
	return true;
}

bool MapSaver::GeoSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	saveFile << "count, time, "
		<< "geo_lat, geo_lon, geo_alt, "
		<< "geo_tim, geo_sat, geo_quality, , " << endl;
	for (Node& node : map->nodes) {
		if (node.geography.llh == vec3_t() || node.time % timeScale != 0
			|| node.geography.numSat < minSatNum || node.geography.quality < minQuality)
			continue;
		saveFile << node.geography.geoCount << ", "
			<< node.time << ", "
			<< FIXEDFLT(R2D * node.geography.llh[0], 0, 6) << ", "
			<< FIXEDFLT(R2D * node.geography.llh[1], 0, 6) << ", "
			<< FIXEDFLT(node.geography.llh[2], 0, 6) << ", "
			<< node.geography.utcTime << ", "
			<< node.geography.numSat << ", "
			<< node.geography.quality << ", " << endl;
	}
	return true;
}

bool MapSaver::MatchSaver() {
	saveFile << "map1, count1, time1, map2, count2, time2, "
		<< "match_id, match_size, match_score, match_reverse, match_nprox, "
		<< "prox1_id, prox2_id, similarity1, similarity2, " << endl;
	for (LoopMatch& lm :* matches)
		for (NodeMatch& nm : lm) {
			if (!nm.proxMatches.size())
				continue;
			saveFile << nm.f->map->id << ", "
				<< nm.f->count << ", "
				<< nm.f->time << ", "
				<< nm.t->map->id << ", "
				<< nm.t->count << ", "
				<< nm.t->time << ", "
				<< lm.id << ", "
				<< lm.size() << ", "
				<< lm.score << ", "
				<< lm.reverse << ", "
				<< nm.proxMatches.size() << ", ";
			for (ProxMatch& pm : nm.proxMatches)
				saveFile << pm.f->id << ", "
					<< pm.t->id << ", "
					<< pm.similarity[0] << ", "
					<< pm.similarity[1] << ", ";
			saveFile << endl;
		}
	return true;
}

bool MapSaver::PoseRefSaver() {
	saveFile << "map1, count1, time1, map2, count2, time2, "
		<< "match_id, match_size, match_score, match_reverse, "
		<< "pose_ref_pos_x, pose_ref_pos_y, pose_ref_pos_z, "
		<< "pose_ref_roll, pose_ref_pitch, pose_ref_yaw, info, " << endl;
	for (LoopMatch& lm : *matches)
		for (NodeMatch& nm : lm) {
			if (nm.locMatch.poseRef == (vec3_t(NAN, NAN, NAN), quat_t(NAN, NAN, NAN, NAN)))
				continue;
			vec3_t rpy = ToRollPitchYaw(nm.locMatch.poseRef.Ori());
			saveFile << nm.f->map->id << ", "
				<< nm.f->count << ", "
				<< nm.f->time << ", "
				<< nm.t->map->id << ", "
				<< nm.t->count << ", "
				<< nm.t->time << ", "
				<< lm.id << ", "
				<< lm.size() << ", "
				<< lm.reverse << ", "
				<< nm.locMatch.poseRef.Pos().X() << ", "
				<< nm.locMatch.poseRef.Pos().Y() << ", "
				<< nm.locMatch.poseRef.Pos().Z() << ", "
				<< FIXEDFLT(rpy.X(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Y(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Z(), 6, 3) << ", ";
			for (int i = 0; i < 6; i++)
				for (int j = i; j < 6; j++)
					saveFile << FIXEDFLT((nm.locMatch.info[i][j] + nm.locMatch.info[j][i]) * 0.5, 6, 3) << ", ";
			saveFile << endl;
		}
	return true;
}

bool MapSaver::AbsProxSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	saveFile << "count, time, "
		<< "abs_prox_x, abs_prox_y, abs_prox_z, " << endl;
	for (Node& node : map->nodes) {
		for (Prox& prox : node.proximities) {
			const vec3_t absProx = node.location.pose * prox.pos;
			saveFile << node.count << ", "
				<< node.time << ", "
				<< FIXEDFLT(absProx.X(), 6, 3) << ", "
				<< FIXEDFLT(absProx.Y(), 6, 3) << ", "
				<< FIXEDFLT(absProx.Z(), 6, 3) << ", " << endl;
		}
		saveFile << endl;
	}
	return true;
}

bool MapSaver::LocGeoSaver(int mapID) {
	Map* map = maps->FindById(mapID);
	if (!map)
		return false;
	Node OrigNode;
	OrigNode.geography.llh = OrigLLH;
	OrigNode.geography.llh_to_ecef();
	saveFile << "count, time, "
		<< "location_pos_x, location_pos_y, location_pos_z, "
		<< "geo_tim, geo_sat, geo_quality, , " << endl;
	for (Node& node : map->nodes) {
		if (node.geography.llh == vec3_t() || node.time % timeScale != 0
			|| node.geography.numSat < minSatNum || node.geography.quality < minQuality)
			continue;
		node.geography.llh_to_ecef();
		node.geography.ecef_to_enu(&(OrigNode.geography));
		node.geography.enu_to_xyz(AngOffset, PosOffset);
		saveFile << node.geography.geoCount << ", "
			<< node.time << ", "
			<< FIXEDFLT(node.geography.xyz.X(), 6, 3) << ", "
			<< FIXEDFLT(node.geography.xyz.Y(), 6, 3) << ", "
			<< FIXEDFLT(node.geography.xyz.Z(), 6, 3) << ", "
			<< node.geography.utcTime << ", "
			<< node.geography.numSat << ", "
			<< node.geography.quality << ", " << endl;
	}
	return true;
}

bool MapSaver::LocMatchSaver() {
	saveFile << "mapID, count, time, "
		<< "location_pos_x, location_pos_y, location_pos_z, "
		<< "location_roll, location_pitch, location_yaw, " << endl << endl;
	for (LoopMatch& lm : *matches)
		for (NodeMatch& nm : lm) {
			vec3_t rpy = ToRollPitchYaw(nm.f->location.pose.Ori());
			saveFile << nm.f->map->id << ", "
				<< nm.f->count << ", "
				<< nm.f->time << ", "
				<< FIXEDFLT(nm.f->location.pose.Pos().X(), 6, 3) << ", "
				<< FIXEDFLT(nm.f->location.pose.Pos().Y(), 6, 3) << ", "
				<< FIXEDFLT(nm.f->location.pose.Pos().Z(), 6, 3) << ", "
				<< FIXEDFLT(rpy.X(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Y(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Z(), 6, 3) << ", " << endl;
			rpy = ToRollPitchYaw(nm.t->location.pose.Ori());
			saveFile << nm.t->map->id << ", "
				<< nm.t->count << ", "
				<< nm.t->time << ", "
				<< FIXEDFLT(nm.t->location.pose.Pos().X(), 6, 3) << ", "
				<< FIXEDFLT(nm.t->location.pose.Pos().Y(), 6, 3) << ", "
				<< FIXEDFLT(nm.t->location.pose.Pos().Z(), 6, 3) << ", "
				<< FIXEDFLT(rpy.X(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Y(), 6, 3) << ", "
				<< FIXEDFLT(rpy.Z(), 6, 3) << ", " << endl << endl;
		}
	return true;
}

bool MapSaver::AbsProxMatchSaver() {
	saveFile << "mapID, count, time, "
		<< "abs_prox_x, abs_prox_y, abs_prox_z, " << endl << endl;
	for (LoopMatch& lm : *matches)
		for (NodeMatch& nm : lm) {
			if (!nm.proxMatches.size())
				continue;
			for (ProxMatch& pm : nm.proxMatches) {
				vec3_t absProx = nm.f->location.pose * pm.f->pos;
				saveFile << nm.f->map->id << ", "
					<< nm.f->count << ", "
					<< nm.f->time << ", "
					<< FIXEDFLT(absProx.X(), 6, 3) << ", "
					<< FIXEDFLT(absProx.Y(), 6, 3) << ", "
					<< FIXEDFLT(absProx.Z(), 6, 3) << ", " << endl;
				absProx = nm.t->location.pose * pm.t->pos;
				saveFile << nm.t->map->id << ", "
					<< nm.t->count << ", "
					<< nm.t->time << ", "
					<< FIXEDFLT(absProx.X(), 6, 3) << ", "
					<< FIXEDFLT(absProx.Y(), 6, 3) << ", "
					<< FIXEDFLT(absProx.Z(), 6, 3) << ", " << endl << endl;
			}
		}
	return true;
}


int MapSaver::Task(int argc, const char* argv[]) {
	int mapNum = 0;
	setting->Get<int>(mapNum, ".mapNum");
	setting->Get<int>(timeScale, ".timeScale");
	setting->Get<int>(minSatNum, ".minSatNum");
	setting->Get<int>(minQuality, ".minQuality");
	setting->Get<vec3_t>(OrigLLH, ".OrigLLH");
	setting->Get<real_t>(AngOffset, ".AngOffset");
	setting->Get<vec2_t>(PosOffset, ".PosOffset");
	string formatLine;
	setting->Get<string>(formatLine, ".saveFormat");
	vector<string> format = split(formatLine, ' ');

	string filename;
	for (int i = 0; i < mapNum; i++) {
		XMLNode* MapSetting;
		try {
			for (int j = 0;; j++) {
				MapSetting = setting->GetNode("Map", j);
				int ID = -1;
				MapSetting->Get<int>(ID, ".ID");
				if (ID == -1 || ID == i + 1)
					break;
			}
		}
		catch (...) { continue; };
		cout << "mapID : " << i + 1 << " - Save : ";

		for (string form : format)
			if (form == "Loc") {
				MapSetting->Get<string>(filename, ".LocFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					LocSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		for (string form : format)
			if (form == "Movement") {
				MapSetting->Get<string>(filename, ".MovementFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					MovementSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		for (string form : format)
			if (form == "Prox") {
				MapSetting->Get<string>(filename, ".ProxFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					ProxSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		for (string form : format)
			if (form == "Geo") {
				MapSetting->Get<string>(filename, ".GeoFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					GeoSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		for (string form : format)
			if (form == "AbsProx") {
				MapSetting->Get<string>(filename, ".AbsProxFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					AbsProxSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		for (string form : format)
			if (form == "LocGeo") {
				MapSetting->Get<string>(filename, ".LocGeoFile");
				if (filename != "") {
					cout << form << " ";
					saveFile.open(filename);
					LocGeoSaver(i + 1);
					saveFile.close();
				}
				break;
			}

		cout << endl;
	}

	cout << "Loop - Save : ";

	for (string form : format)
		if (form == "Match") {
			setting->Get<string>(filename, ".MatchFile");
			if (filename != "") {
				cout << form << " ";
				saveFile.open(filename);
				MatchSaver();
				saveFile.close();
			}
			break;
		}

	for (string form : format)
		if (form == "PoseRef") {
			setting->Get<string>(filename, ".PoseRefFile");
			if (filename != "") {
				cout << form << " ";
				saveFile.open(filename);
				PoseRefSaver();
				saveFile.close();
			}
			break;
		}

	for (string form : format)
		if (form == "LocMatch") {
			setting->Get<string>(filename, ".LocMatchFile");
			if (filename != "") {
				cout << form << " ";
				saveFile.open(filename);
				LocMatchSaver();
				saveFile.close();
			}
			break;
		}

	for (string form : format)
		if (form == "AbsProxMatch") {
			setting->Get<string>(filename, ".AbsProxMatchFile");
			if (filename != "") {
				cout << form << " ";
				saveFile.open(filename);
				AbsProxMatchSaver();
				saveFile.close();
			}
			break;
		}

	cout << endl;

	return 0;
}

}