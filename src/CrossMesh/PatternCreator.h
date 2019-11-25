///////////////////////////////////////////////////////////////
//
// MeshCreator.h
//
//   Create Polygonal Meshes
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_CREATOR_H
#define _MESH_CREATOR_H

#include "Utility/vec.h"
#include <vector>
#include "Mesh/CrossMesh.h"

typedef shared_ptr<PolyMesh> pPolyMesh;
typedef shared_ptr<_Polygon> pPolygon;

using namespace std;

class PatternCreator : public TopoObject
{
public:
	PatternCreator();
	PatternCreator(shared_ptr<gluiVarList> var):TopoObject(var){}
	~PatternCreator();

public:

	// Create Mesh from Multiple Meshes
	void CreateMesh_Merge(vector<pPolyMesh> polyMeshes, pPolyMesh &out);

	// Create Mesh (2D Regular Pattern)
	void CreateMesh_2DPattern(int patternID,
	                          int patternRadius,
	                          shared_ptr<CrossMesh> &out);


public:
	void CreateMesh_2DPattern(vector<_Polygon> &root_polys,
                              Vector3f DX,
                              Vector3f DY,
                              int Nx,
                              int Ny,
                              shared_ptr<CrossMesh> &out);

	void CreateMesh_2DPattern_HexagonPattern2(int patternRadius, shared_ptr<CrossMesh> &out);
    void CreateMesh_2DPattern_HexagonPattern3(int patternRadius, shared_ptr<CrossMesh> &out);
    void CreateMesh_2DPattern_PentagonPattern3(int patternRadius, shared_ptr<CrossMesh> &out);
	int GetPolygonIndexInList(pPolygon tagtPoly, vector<pPolygon> polyList);

public:

	// Compute Neighbors
	void ComputeNeighbors(int crosstype, pPolygon poly, vector<pPolygon> &out);
	void ComputeNeighbor_Square(pPolygon poly, pPolygon &out);
	void ComputeNeighbor_Hexagon(pPolygon poly, pPolygon &out);
	void ComputeNeighbor_Octagon_Square(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Hexagon_Rhombus(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Dodecagon(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Dodecagon_Hexagon_Square(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Square_Rhombus(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Cross(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Snow(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Mirror(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Rhombus(pPolygon poly, int edgeID, pPolygon &out);

    void ComputeNeighbor_Octagon_Square_Colinear(pPolygon poly, int edgeID, pPolygon &out);

	Vector3f ComputeTileTranslation(pPolygon neighbor, Vector3f tagtStaPt, Vector3f tagtEndPt);
    Vector3f ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(pPolygon poly, int edgeID);

	// Create Polygons
	void CreatePolygon_Root(int edgeNum, float edgeLen, pPolygon &out);
	void CreatePolygon_Square(pPolygon &out, float edgeLen, int polyType = POLY_SQUARE_THETA_45);
	void CreatePolygon_Hexagon(pPolygon &out, float edgeLen, int polyType = POLY_HEXAGON_TYPE_0);
	void CreatePolygon_Octagon(pPolygon &out, float edgeLen, int polyType);
	void CreatePolygon_Dodecagon(pPolygon &out, float edgeLen);
	void CreatePolygon_Rhombus(pPolygon &out, float edgeLen, int polyType = POLY_RHOMBUS_THETA_0);

	void CreatePolygon_Pentagon_Cross(pPolygon &out, float edgeLen, int polyType = POLY_PENTAGON_CROSS_TYPE_0);
	void CreatePolygon_Pentagon_Snow(pPolygon &out, float edgeLen, int polyType = POLY_PENTAGON_SNOW_TYPE_0);
    void CreatePolygon_Pentagon_Mirror(pPolygon &out, float edgeLen, int polyType = POLY_PENTAGON_MIRROR_TYPE_0);

	// Create Mesh (a Single Fan)
	void CreateMesh_Fan(Vector3f edgeA, Vector3f edgeB, Vector3f edgeMid, pPolyMesh &out);
	void CreatePolygon_Fan(Vector3f verA, Vector3f verB, Vector3f verM, pPolygon &out);

public:
    //python
    vector<vector<double>> PyCreateMesh_2DPattern(int patternID, int patternRadius);

};

#endif