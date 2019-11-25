///////////////////////////////////////////////////////////////
//
// BodyMobili.h
//
//   Evaluate Part or Part Group Mobility
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Jan/2018
//
///////////////////////////////////////////////////////////////


#ifndef _BODY_MOBILI_H
#define _BODY_MOBILI_H

#include <vector>
#include "../Utility/vec.h"
#include "../Utility/HelpStruct.h"

using namespace std;

class PolyMesh;
typedef shared_ptr<HypPlane> pHypPlane;
typedef shared_ptr<HypEdge> pHypEdge;
typedef shared_ptr<HypVertex> pHypVertex;
typedef shared_ptr<PolyMesh> pPolyMesh;

class BodyMobili
{
public:
	vector<Vector3f> planeNormals;          // Input plane normals

	vector<pHypPlane> faceList;             // Constructed faces
	vector<pHypEdge> edgeList;              // Constructed edges
	vector<pHypVertex> verList;             // Constructed vertices

	pPolyMesh mobiliMesh;                   // Mobility solution space
	Vector3f mobiliVec;                     // Mobility average direction
	float mobiliScore;                      // Mobility measurement

	vector<Vector3f> mobiliRays;            // Mobility extreme rays (start at (0,0,0)) (Note: they have not been normalized)

	float stabiliScore;                     // Stability measurement (relative to the gravity)

public:

	float bodyScale;                        // Scale of tested primitives (for rendering only)
	Vector3f bodyTrans;                     // Translation of tested primitives (for rendering only)

public:
	BodyMobili(vector<Vector3f> _planeNormals, float _bodyScale, Vector3f _bodyTrans);
	~BodyMobili();
	void Clear();

	// Evaluate Body Mobility
	bool EvaluateBodyMobility();
	void ComputeMobilityVector();
	void ComputeStabilityScore();

	// Compute Faces and Edges
	void ComputeFaces();
	void ComputeEdges(vector<pHypPlane>& _planeList);
	void PlanePlaneIntersect(pHypPlane faceA, pHypPlane faceB, pHypEdge &out);
	bool EdgeHitFace(Vector3f edgePt, Vector3f edgeDir, pHypPlane face, Vector3f &hitPt);

	// Compute Vertices
	void ComputeVertices(vector<pHypEdge>& _edgeList);
	bool IsPointInList(Vector3f tagtPoint, vector<pHypVertex>& _verList);

	// Validate Vertices
	vector<Vector3f> ValidateVertices();
	bool IsValidVertex(Vector3f point);

	void ComputeMobility(vector<Vector3f> pointList);
	Vector3f GetPlaneVertex(Vector3f edgeVertex, Vector3f planeNormal);
	vector<Vector3f> SortValidVertices(vector<Vector3f> pointList);
	void ComputeMobilityMesh(vector<Vector3f> sortedPoints);
	void CorrectMobilityMesh(Vector3f _mobiliVec);
	void ComputeMobilityScore(vector<Vector3f> sortedPoints);

#if USE_OPENGL_DRAW
	// Draw Construction
	void DrawMobilityFaces();
	void DrawMobilityFace(pHypPlane plane);
	void DrawMobilityEdges();
	void DrawMobilityEdge(pHypEdge edge, float length);
	void DrawMobilityVertices();
	void DrawMobilityVertex(pHypVertex vertex);

	// Draw Mobility
	void DrawMobilityRay();
	void DrawMobilityVector(float lineWidth, Vector3f color);
	void DrawMobilityMesh();
#endif
};

#endif