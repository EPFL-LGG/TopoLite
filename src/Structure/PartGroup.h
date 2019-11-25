///////////////////////////////////////////////////////////////
//
// PartGroup.h
//
//   A group of parts
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Jan/2018
//
///////////////////////////////////////////////////////////////


#ifndef _PART_GROUP_H
#define _PART_GROUP_H


#include <vector>
#include "../Utility/vec.h"


using namespace std;

class BodyMobili;


class PartGroup 
{
public:
	vector<wpPart> groupParts;           // A group of parts

	vector<weak_ptr<OrientPoint>> oriPoints;      // Oriented points on part group boundary
	 
	pBodyMobili groupMobili;            // Mobility of the part group


public:
	PartGroup(vector<pPart> partList, vector<int> groupPartIDs);
	PartGroup(const vector<wpPart>& _groupParts);
	PartGroup(vector<wpPart> partList, vector<int> groupPartIDs);
	~PartGroup();

	// Identify Group Boundary
	void IdentifyGroupBoundary();
	int GetPartIndexInList(pPart tagtPart, vector<wpPart> partList);

	// Evaluate Group Mobility
	bool EvaluateGroupMobility();
	vector<Vector3f> ComputePlaneNormals();
	Box ComputeBBox();
	Vector3f ComputeCentroid();

	// Write OBJ Model
	void WriteGroupOBJModel(const char *objFileName, bool triangulate = false);
	void WriteGroupOBJWireModel(char *objFileName);

	vector<Vector3f> GetAllVertices();
	vector<vector<int>> GetAllPolygons(vector<Vector3f> allVertices);

public:
#if USE_OPENGL_DRAW
	// Draw Group Mobility
	void DrawMobiliFaces();
	void DrawMobiliEdges();
	void DrawMobiliVertices();
	void DrawMobiliRays();
	void DrawMobiliVector();
	void DrawMobiliMesh();

	// Draw Debug
	void DrawOriPoints();
#endif
};

#endif