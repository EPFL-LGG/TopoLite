///////////////////////////////////////////////////////////////
//
// PartGeom.h
//
//   Construct Part Geometry
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Dec/2017
//
///////////////////////////////////////////////////////////////


#ifndef _PART_GEOM_H
#define _PART_GEOM_H

#include <vector>
#include <Eigen/Dense>

#include "TopoLite/Mesh/Cross.h"
#include "TopoLite/Mesh/Polygon.h"
#include "TopoLite/Mesh/PolyMesh.h"

#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/HelpStruct.h"
#include "TopoLite/Utility/ConvexHull2D.h"
#include "TopoLite/Utility/TopoObject.h"

using namespace std;
using pHypPlane = shared_ptr<HypPlane>;
using pHypEdge =  shared_ptr<HypEdge> ;
using pHypVertex =  shared_ptr<HypVertex> ;
using pPolyMesh =  shared_ptr<PolyMesh> ;

class PartGeom : public TopoObject
{
public:
	//in
	vector<Vector3f> polygon;                    // Input cross polygon
	vector<weak_ptr<OrientPoint>> oriPoints;     // Input oriented points
	weak_ptr<Cross> cross;

	//out
	vector<pHypPlane> hypList;                  // Constructed hyper plane
	vector<pHypEdge> edgeList;                  // Constructed edges
	vector<pHypVertex> verList;                 // Constructed vertices
	vector<shared_ptr<_Polygon>> faceList;        // Constructed face
public:

	PartGeom(shared_ptr<Cross> _cross, shared_ptr<InputVarList> var);
	PartGeom(const PartGeom &_geom);
	~PartGeom();

	void Clear();
	void ParseCrossData(shared_ptr<Cross> _cross);

public:

	// Validate Part Geometry
	bool ValidateTiltNormal();
	bool IsLegalGeometry();

	// Compute Part Geometry
    void ComputePartGeometry(Vector2f cutPlaneHeight, pPolyMesh &polyMesh);
    void ComputeValidVertices(vector<Vector3f> &pointList);
    void ComputeVertices();
    void ComputeFaces(Vector2f cutPlaneHeight);
	void Convert2PolyMesh(vector<Vector3f> &ver, vector<Vector3i> &tri, pPolyMesh &polyMesh);
	bool IsValidVertex(Vector3f point);
};

#endif