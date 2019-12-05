///////////////////////////////////////////////////////////////
//
// Cross.h
//
//   Cross section
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 15/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _CROSS_MESH_H
#define _CROSS_MESH_H

#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/TopoObject.h"

#include "TopoLite/Mesh/PolyMesh.h"

#include "Cross.h"



#include <vector>

using namespace std;
using pCross = shared_ptr<Cross>;
using wpCross = weak_ptr<Cross>;

class CrossMesh : public TopoObject
{

public:

	vector<pCross> crossList;                // Cross list of the mesh

	vector<Vector3f> vertexList;						// all vertices point of cross

	vector<vector<wpCross>> vertexCrossList; 	// all cross around a vertex

public:

	//int boundary_vertexList_index;						//seperate the boundary vertices which will not optimize in the shapeop function

	shared_ptr<PolyMesh> baseMesh2D;

public:

    CrossMesh(std::shared_ptr<InputVarList> var);
    CrossMesh(const CrossMesh &_cross);
    ~CrossMesh();

    bool checkCoherency();
    void Print();
	void SetBaseMesh2D(shared_ptr<PolyMesh> _baseMesh2D);
    vector<Vector3f> GetAllVertices();

public:

	void UpdateCrossFromVertexList();
	void UpdateCrossVertexIndex();
	void updateCrossID();
    float averageCrossSize();
    void TranslateMesh(Vector3f mv);
    shared_ptr<PolyMesh> getPolyMesh();

public:
	// Save OBJ file
	void WriteOBJModel(const char *objFileName);
};

#endif

