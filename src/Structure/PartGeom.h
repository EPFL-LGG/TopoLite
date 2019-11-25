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

#include "Mesh/Cross.h"
#include "Utility/vec.h"
#include "Utility/HelpStruct.h"
#include "Utility/ConvexHull2D.h"
#include "Mesh/Polygon.h"

using namespace std;
struct OrientPoint;
class PolyMesh;
using pHypPlane = shared_ptr<HypPlane>;
typedef shared_ptr<HypEdge> pHypEdge;
typedef shared_ptr<HypVertex> pHypVertex;
typedef shared_ptr<PolyMesh> pPolyMesh;
class PartGeom 
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

	PartGeom(shared_ptr<Cross> _cross);
	PartGeom(const PartGeom &_geom);
	~PartGeom();

	void Clear();
	void UpdateCross(shared_ptr<Cross> _cross);

	// Compute Part Geometry
	void ComputePartGeometry(Vector2f cutPlaneHeight, pPolyMesh &polyMesh);
	bool ValidateTiltNormal();
	bool IsFiniteIntersection();
	Vector3f GetPseudoOrigin();
	void ComputePartGeometry_Matrix();
	bool IsLegalGeometry();

	// Compute Faces and Edges
	void ComputeFaces(Vector2f cutPlaneHeight);
	void ComputeEdges(vector<pHypPlane> &_planeList);
	void PlanePlaneIntersect(pHypPlane faceA, pHypPlane faceB, pHypEdge &out);
	bool EdgeHitFace(Vector3f edgePt, Vector3f edgeDir, pHypPlane plane, Vector3f &hitPt);
	void Triangles2PolyMesh(vector<Vector3f> &ver, vector<Vector3i> &tri, pPolyMesh &polyMesh);

	// Compute Vertices
	void ComputeVertices(vector<pHypPlane> &_faceList, vector<pHypEdge> &_edgeList);
	bool IsPointInList(Vector3f tagtPoint, vector<pHypVertex> &_verList);

	// Validate Vertices
	void ValidateVertices(vector<Vector3f> &pointList);
	bool IsValidVertex(Vector3f point);

    vector<Vector3f> verDiffs;

#if USE_OPENGL_DRAW
public:

	// Draw Base Polygon
	void DrawInnerPolygon();
	void DrawOriPoints();
	void DrawVerDiffs();


	// Draw Part Construction
	void DrawFaces();
	void DrawFace(pHypPlane plane);
	void DrawEdges();
	void DrawEdge(pHypEdge edge, float length);
	void DrawVertices();
	void DrawVertex(pHypVertex vertex);
#endif
};

#endif