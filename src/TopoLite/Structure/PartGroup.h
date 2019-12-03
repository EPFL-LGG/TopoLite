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
#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/TopoObject.h"

using namespace std;

class PartGroup :public TopoObject
{
public:
	vector<wpPart> groupParts;           // A group of parts

	vector<weak_ptr<OrientPoint>> oriPoints;      // Oriented points on part group boundary

public:

	PartGroup(vector<pPart> partList, vector<int> groupPartIDs, shared_ptr<InputVarList> var);
	PartGroup(vector<wpPart> partList, vector<int> groupPartIDs, shared_ptr<InputVarList> var);
    PartGroup(const vector<wpPart>& _groupParts, shared_ptr<InputVarList> var);
	~PartGroup();

	vector<Vector3f> GetAllVertices();
	vector<vector<int>> GetAllPolygons(const vector<Vector3f> &allVertices);

public:
    // Write OBJ Model
    void WriteGroupOBJModel(const char *objFileName, bool triangulate = false);
    void WriteGroupOBJWireModel(char *objFileName);
};

#endif