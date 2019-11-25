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
#include "Utility/vec.h"


using namespace std;

class PartGroup 
{
public:
	vector<wpPart> groupParts;           // A group of parts

	vector<weak_ptr<OrientPoint>> oriPoints;      // Oriented points on part group boundary

public:

	PartGroup(vector<pPart> partList, vector<int> groupPartIDs);
	PartGroup(vector<wpPart> partList, vector<int> groupPartIDs);
    PartGroup(const vector<wpPart>& _groupParts);
	~PartGroup();

	vector<Vector3f> GetAllVertices();
	vector<vector<int>> GetAllPolygons(const vector<Vector3f> &allVertices);

public:
    // Write OBJ Model
    void WriteGroupOBJModel(const char *objFileName, bool triangulate = false);
    void WriteGroupOBJWireModel(char *objFileName);
};

#endif