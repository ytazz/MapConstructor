#include <iostream>
#include <iomanip>
#include <fstream>
#include <conio.h>
#include <sstream>
#include <time.h>
#include <string.h>
#include <array>
#include "map_loader.h"

#include "sbtypes.h"
#include "sbrollpitchyaw.h"
#include "sbconverter.h"
#include "sbxml.h"

using namespace std;
using namespace Scenebuilder;

namespace MapConstructor {

bool MapLoader::LocLoader(const int mapID) {
	// Find a map with mapID registered (if not, build a new one)
	Map* map(maps->FindById(mapID));
	if (!map) {
		map = new Map();
		maps->push_back(map);
		map->id = mapID;
	}

	for (int r = 0; r < loadFile.NumRow(); r++) {
		const int count = loadFile.Get<int>(r, 0);
		if (count <= 0)
			continue;

		// Find a node with count registered (if not, build a new one)
		Node* node(map->nodes.FindByCount(count));
		if (!node) {
			node = new Node();
			map->nodes.push_back(node);
			node->map = map;
			node->count = loadFile.Get<int>(r, 0);
			node->time = loadFile.Get<int>(r, 1);
			node->index = NodeId++;
		}

		// Register robot pose
		node->location.pose.Pos() = vec3_t{ loadFile.Get<real_t>(r, 2), loadFile.Get<real_t>(r, 3), loadFile.Get<real_t>(r, 4) };
		node->location.pose.Ori() = FromRollPitchYaw(vec3_t{ loadFile.Get<real_t>(r, 5), loadFile.Get<real_t>(r, 6), loadFile.Get<real_t>(r, 7) });
	}
	return true;
}

bool MapLoader::MovementLoader(const int mapID) {
	// Find a map with mapID registered (if not, build a new one)
	Map* map(maps->FindById(mapID));
	if (!map) {
		map = new Map();
		maps->push_back(map);
		map->id = mapID;
	}

	for (int r = 0; r < loadFile.NumRow(); r++) {
		const int count = loadFile.Get<int>(r, 0);
		if (count <= 0)
			continue;

		// Find a node with count registered (if not, build a new one)
		Node* node(map->nodes.FindByCount(count));
		if (!node) {
			node = new Node();
			map->nodes.push_back(node);
			node->map = map;
			node->count = loadFile.Get<int>(r, 0);
			node->time = loadFile.Get<int>(r, 1);
			node->index = NodeId++;
		}

		// Find a prevNode with count registered (if not, build a new one)
		Node* prevNode(map->nodes.FindByCount(count - 1));
		if (!prevNode) {
			prevNode = new Node();
			map->nodes.push_back(prevNode);
			prevNode->map = map;
			prevNode->count = loadFile.Get<int>(r, 0);
			prevNode->time = loadFile.Get<int>(r, 1);
			prevNode->index = NodeId++;
		}

		// Register odometry
		node->movement.prevNode = prevNode;
		node->movement.poseRef.Pos() = vec3_t{ loadFile.Get<real_t>(r, 2), loadFile.Get<real_t>(r, 3), loadFile.Get<real_t>(r, 4) };
		node->movement.poseRef.Ori() = FromRollPitchYaw(vec3_t{ loadFile.Get<real_t>(r, 5), loadFile.Get<real_t>(r, 6), loadFile.Get<real_t>(r, 7) });
		for (int i = 0, k = 0; i < 6; i++)
			for (int j = i; j < 6; j++, k++)
				node->movement.info[i][j] = node->movement.info[j][i] = loadFile.Get<real_t>(r, 8 + k);
	}
	return true;
}

bool MapLoader::ProxLoader(const int mapID) {
	// Find a map with mapID registered (if not, build a new one)
	Map* map(maps->FindById(mapID));
	if (!map) {
		map = new Map();
		maps->push_back(map);
		map->id = mapID;
	}

	for (int r = 0; r < loadFile.NumRow(); r++) {
		const int count = loadFile.Get<int>(r, 0);
		if (count <= 0)
			continue;

		Node* node(map->nodes.FindByCount(count));
		// Find a node with count registered (if not, build a new one)
		if (!node) {
			node = new Node();
			map->nodes.push_back(node);
			node->map = map;
			node->count = loadFile.Get<int>(r, 0);
			node->time = loadFile.Get<int>(r, 1);
			node->index = NodeId++;
		}

		int n = loadFile.Get<int>(r, 2);
		for (int k = 0; k < n; k++) {
			// Find a proximity-point with Id registered (if not, build a new one)
			Prox* prox(node->proximities.FindById(loadFile.Get<int>(r, 3 + 6 * k)));
			if (!prox) {
				prox = new Prox();
				node->proximities.push_back(prox);
				prox->node = node;
				prox->index = node->proximities.size() - 1;
				prox->id = loadFile.Get<int>(r, 3 + 6 * k);
			}

			// Register proximity-point data
			prox->duration = loadFile.Get<int>(r, 4 + 6 * k);
			prox->velocity = loadFile.Get<real_t>(r, 5 + 6 * k);
			prox->spherical = vec3_t{ loadFile.Get<real_t>(r, 6 + 6 * k), loadFile.Get<real_t>(r, 7 + 6 * k), loadFile.Get<real_t>(r, 8 + 6 * k) };
			prox->sph_to_xyz();
		}
	}
	return true;
}

bool MapLoader::GeoLoader(const int mapID) {
	// Find a map with mapID registered (if not, build a new one)
	Map* map(maps->FindById(mapID));
	if (!map) {
		map = new Map();
		maps->push_back(map);
		map->id = mapID;
	}

	int timeStep = 500;
	setting->Get<int>(timeStep, ".timeStep");

	for (int r = 0; r < loadFile.NumRow(); r++) {
		const int count = loadFile.Get<int>(r, 0);
		const int time = loadFile.Get<int>(r, 1);
		if (count <= 0)
			continue;

		Node* node(map->nodes.FindByCount(count));
		if (time == count * timeStep) {
			// Find a node with count registered (if not, build a new one)
			if (!node) {
				node = new Node();
				map->nodes.push_back(node);
				node->map = map;
				node->count = -1;
				node->time = time;
				node->index = NodeId++;
			}
		}
		else {
			// Find a node with time registered (if not, don't register Geo data)
			node = map->nodes.FindByTime(time);
			if (!node) {
				if (time % timeStep == 0) {
					node = new Node();
					map->nodes.push_back(node);
					node->map = map;
					node->count = -1;
					node->time = time;
					node->index = NodeId++;
				}
				else
					continue;
			}
		}

		// Register Geo data
		node->geography.geoCount = count;
		node->geography.llh = vec3_t{ D2R * loadFile.Get<real_t>(r, 2), D2R * loadFile.Get<real_t>(r, 3), loadFile.Get<real_t>(r, 4) };
		node->geography.utcTime = loadFile.Get<int>(r, 5);
		node->geography.numSat = loadFile.Get<int>(r, 6);
		node->geography.quality = loadFile.Get<int>(r, 7);
	}
	return true;
}

bool MapLoader::PointCloudLoader(const string filename, const int mapID) {
	// Find a map with mapID registered (if not, build a new one)
	Map* map(maps->FindById(mapID));
	if (!map) {
		map = new Map();
		maps->push_back(map);
		map->id = mapID;
	}

	ifstream PCFile(filename);
	for (int r = 0;; r++) {
		string str;
		streampos g = PCFile.tellg();
		if (!getline(PCFile, str))
			break;
		string str0;
		getline(stringstream(str), str0, ',');
		const int count = atoi(str0.c_str());
		if (count <= 0)
			continue;

		// Find a node with count registered (if not, don't register pointcloud data)
		Node* node(map->nodes.FindByCount(count));
		if (!node)
			continue;

		// Register the file name and first character position where the pointcloud data is located
		node->pc.fileName = filename;
		node->pc.row = g;
	}
	return true;
}

bool MapLoader::MatchLoader() {
	// submap0, count0, time0, submap1, count1, time1, index, size, score, reverse, [prox0, prox1, sim0, sim1] ...
	for (int r = 0; r < loadFile.NumRow(); r++) {
        int ncol = loadFile.NumCol(r);
        if(ncol < 10)
            continue;

        int c = 0;

		int mapID[2] = { loadFile.Get<int>(r, c+0), loadFile.Get<int   >(r, c+3) };
		int count[2] = { loadFile.Get<int>(r, c+1), loadFile.Get<int   >(r, c+4) };
		int time[2]  = { loadFile.Get<int>(r, c+2), loadFile.Get<int   >(r, c+5) };
		int index    = loadFile.Get<int>   (r, c+6);
		int size     = loadFile.Get<int>   (r, c+7);
		real_t score = loadFile.Get<real_t>(r, c+8);
		bool reverse = loadFile.Get<bool>  (r, c+9);
		
        c = 10;

        //const int num_pm = loadFile.Get<int>(r, 10);
        int num_pm = (ncol - c)/4;

		// Find 2 maps with 2 mapIDs registered (if not, don't register match data)
		Map* fmap = maps->FindById(mapID[0]);
		Map* tmap = maps->FindById(mapID[1]);
		if (!fmap || !tmap)
			continue;

		// Find 2 nodes with 2 counts registered (if not, don't register match data)
		Node* fnode = fmap->nodes.FindByCount(count[0]);
		Node* tnode = tmap->nodes.FindByCount(count[1]);
		if (!fnode || !tnode)
			continue;

		// Find a loop section with matchID registered (if not, build a new one)
		LoopMatch* lm = matches->FindById(index);
		if (!lm) {
			lm = new LoopMatch();
			matches->push_back(lm);
			lm->id = index;
			lm->score = score;
			lm->reverse = reverse;
		}

		// Find a node pair with 2 nodes registered (if not, build a new one)
		NodeMatch* nm = lm->FindByNodes(fnode, tnode);
		if (!nm) {
			nm = new NodeMatch(fnode, tnode);
			lm->push_back(nm);
		}

		for (int k = 0; k < num_pm; k++) {
			const int proxId[2] = { loadFile.Get<int>   (r, c + 4*k + 0), loadFile.Get<int>   (r, c + 4*k + 1) };
			const real_t sim[2] = { loadFile.Get<real_t>(r, c + 4*k + 2), loadFile.Get<real_t>(r, c + 4*k + 3) };

			// Find 2 proxs with 2 Ids registered (if not, don't register prox pair)
			Prox* fprox = fnode->proximities.FindById(proxId[0]);
			Prox* tprox = tnode->proximities.FindById(proxId[1]);
			if (!fprox || !tprox)
				continue;

			//// ignored if similarity is below threshold
			//if (sim[0] < minProxMatchSimilarity || sim[1] < minProxMatchSimilarity)
			//	continue;

			//real_t x[2] = { fprox->pos.X(), tprox->pos.X() };
			//real_t y[2] = { fprox->pos.Y(), tprox->pos.Y() };
			//// ignored if yaw difference is above threshold
			//real_t posDiff = sqrt((x[1] - x[0]) * (x[1] - x[0]) + (y[1] - y[0]) * (y[1] - y[0]));
			//if (posDiff > maxPosMatchError)
			//	continue;

			//real_t yaw[2] = { fprox->spherical[0], tprox->spherical[0] };
			//real_t yawDiff = (reverse ? M_PI : 0.0) + yaw[1] - yaw[0];
			//yawDiff = WrapPi(yawDiff);
			//if (std::abs(yawDiff) > maxYawMatchError)
			//	continue;

			// Find a prox pair with 2 proxs registered (if not, build a new one)
			ProxMatch* pm = nm->proxMatches.FindByProxs(fprox, tprox);
			if (!pm) {
				pm = new ProxMatch(fprox, tprox);
				nm->proxMatches.push_back(pm);
				pm->similarity[0] = sim[0];
				pm->similarity[1] = sim[1];
			}
		}
	}
	return true;
}

bool MapLoader::PoseRefLoader() {
	for (int r = 0; r < loadFile.NumRow(); r++) {
		// count0, time0, id0, count1, time1, id1
		const int mapID[2] = { loadFile.Get<int>(r, 0), loadFile.Get<int   >(r, 3) };
		const int count[2] = { loadFile.Get<int>(r, 1), loadFile.Get<int   >(r, 4) };
		const int time[2] = { loadFile.Get<int>(r, 2), loadFile.Get<int   >(r, 5) };
		const int index = loadFile.Get<int>(r, 6);
		const int size = loadFile.Get<int>(r, 7);
		const bool reverse = loadFile.Get<bool>(r, 8);

		// Find 2 maps with 2 mapIDs registered (if not, don't register match data)
		Map* fmap = maps->FindById(mapID[0]);
		Map* tmap = maps->FindById(mapID[1]);
		if (!fmap || !tmap)
			continue;

		// Find 2 nodes with 2 counts registered (if not, don't register match data)
		Node* fnode = fmap->nodes.FindByCount(count[0]);
		Node* tnode = tmap->nodes.FindByCount(count[1]);
		if (!fnode || !tnode)
			continue;

		// Find a loop section with matchID registered (if not, build a new one)
		LoopMatch* lm = matches->FindById(index);
		if (!lm) {
			lm = new LoopMatch();
			matches->push_back(lm);
			lm->id = index;
			lm->score = 0.;
			lm->reverse = reverse;
		}

		// Find a node pair with 2 nodes registered (if not, build a new one)
		NodeMatch* nm = lm->FindByNodes(fnode, tnode);
		if (!nm) {
			nm = new NodeMatch(fnode, tnode);
			lm->push_back(nm);
		}

		// Register relative pose and information matrix
		nm->locMatch;
		nm->locMatch.poseRef.Pos() = vec3_t{ loadFile.Get<real_t>(r, 9), loadFile.Get<real_t>(r, 10), loadFile.Get<real_t>(r, 11) };
		nm->locMatch.poseRef.Ori() = FromRollPitchYaw(vec3_t{ loadFile.Get<real_t>(r, 12), loadFile.Get<real_t>(r, 13), loadFile.Get<real_t>(r, 14) });
		for (int i = 0, k = 0; i < 6; i++)
			for (int j = i; j < 6; j++, k++)
				nm->locMatch.info[i][j] = nm->locMatch.info[j][i] = loadFile.Get<real_t>(r, 15 + k);
	}
	return true;
}


int MapLoader::Task(int argc, const char* argv[]) {
	string delim = ",";
	setting->Get<string>(delim, ".delim");
	int mapNum = 0;
	setting->Get<int>(mapNum, ".mapNum");
	string formatLine;
	setting->Get<string>(formatLine, ".loadFormat");
	vector<string> format = split(formatLine, ' '); // Set the format to load

	string filename;
	for (int i = 0; i < mapNum; i++) {
		// Load settings for each map data
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
		cout << "mapID : " << i + 1 << " - Load : ";

		filename = "";
		for(string form : format)
			if (form == "Loc") { // Register robot pose
				MapSetting->Get<string>(filename, ".LocFile");
				if (filename != "") {
					cout << form << " ";
                    loadFile.Clear();
					loadFile.Read(filename, delim);
					LocLoader(i + 1);
				}
				break;
			}

		filename = "";
		for (string form : format)
			if (form == "Movement") { // Register odometry
				MapSetting->Get<string>(filename, ".MovementFile");
				if (filename != "") {
					cout << form << " ";
                    loadFile.Clear();
					loadFile.Read(filename, delim);
					MovementLoader(i + 1);
				}
				break;
			}

		filename = "";
		for (string form : format)
			if (form == "Prox") { // Register proximity-point position
				MapSetting->Get<string>(filename, ".ProxFile");
				if (filename != "") {
					cout << form << " ";
                    loadFile.Clear();
					loadFile.Read(filename, delim);
					ProxLoader(i + 1);
				}
				break;
			}

		filename = "";
		for (string form : format)
			if (form == "Geo") { // Register RTK positioning data
				MapSetting->Get<string>(filename, ".GeoFile");
				if (filename != "") {
					cout << form << " ";
                    loadFile.Clear();
					loadFile.Read(filename, delim);
					GeoLoader(i + 1);
				}
				break;
			}

		filename = "";
		for (string form : format)
			if (form == "PC") { // Register where point cloud data is located
				MapSetting->Get<string>(filename, ".PCFile");
				if (filename != "") {
					cout << form << " ";
					PointCloudLoader(filename, i + 1);
				}
				break;
			}

		cout << endl;
	}

	cout << "Loop - Load : ";

	filename = "";
	for (string form : format)
		if (form == "Match") { // Register loop constraint by proximity-point pair
			setting->Get<string>(filename, ".MatchFile");
			if (filename != "") {
				cout << form << " ";
                loadFile.Clear();
				loadFile.Read(filename, delim);
				MatchLoader();
			}
			break;
		}

	filename = "";
	for (string form : format)
		if (form == "PoseRef") { // Register loop constraint relative pose
			setting->Get<string>(filename, ".PoseRefFile");
			if (filename != "") {
				cout << form << " ";
                loadFile.Clear();
				loadFile.Read(filename, delim);
				PoseRefLoader();
			}
			break;
		}

	cout << endl;

	return 0;
}

}