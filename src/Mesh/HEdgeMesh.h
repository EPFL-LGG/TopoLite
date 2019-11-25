///////////////////////////////////////////////////////////////
//
// HEdgeMesh.h
//
//   Half-Edge Mesh Structure
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 10/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef _HALF_EDGE_MESH_H
#define _HALF_EDGE_MESH_H

#include "../Utility/vec.h"
#include <vector>

using namespace std;


class PolyMesh;

class HVertex;
class HEdge;
class HFace;

typedef shared_ptr<PolyMesh> pPolyMesh;

typedef shared_ptr<HVertex> pHVertex;
typedef weak_ptr<HVertex> wpHVertex;

typedef shared_ptr<HEdge> pHEdge;
typedef weak_ptr<HEdge> wpHEdge;

typedef shared_ptr<HFace> pHFace;
typedef weak_ptr<HFace> wpHFace;



//**************************************************************************************//
//                                  Vertex, Edge, Face
//**************************************************************************************//

class HVertex
{
public:
	int id;                          // Vertex ID
	Vector3f point;                  // Vertex position 
	Vector3f normal;                 // Vertex normal 

	vector<wpHEdge> outEdges;         // All the outgoing edges from vertex
};

class HEdge
{
public:
	int id;                          // Half edge ID
	wpHVertex staVer;                 // Vertex at starting point
	wpHVertex endVer;                 // Vertex at ending point

	wpHEdge next;                     // Next half edge
	wpHEdge prev;                     // Previous half edge
	wpHEdge twin;                     // Opposite half edge

	wpHFace face;                     // Incident face

public:
	HEdge()
	{
	}

	bool IsEqual(pHEdge edge)
	{
		if (this->id         == edge->id &&
			this->staVer.lock()->id == edge->staVer.lock()->id &&
			this->endVer.lock()->id == edge->endVer.lock()->id)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
};

class HFace
{
public:
	int id;                          // Face ID 
	Vector3f normal;                 // Face normal

	vector<wpHVertex> vertices;       // Face vertices
	wpHEdge startEdge;                // Starting edge of face

	vector<wpHEdge> edges;            // All vertices on the face, saving in counter-clockwise
};




//**************************************************************************************//
//                                   Half-Edge Mesh 
//**************************************************************************************//

class HEdgeMesh
{
public:

	vector<shared_ptr<HVertex>>  vertexList;    // Vertex list for half edge mesh
	vector<shared_ptr<HFace>>    faceList;      // Face list for half edge mesh
	vector<shared_ptr<HEdge>>    edgeList;      // Edge list for half edge mesh

public:
	HEdgeMesh();
	~HEdgeMesh();
	void ClearHEdgeMesh();
	void InitHEdgeMesh(vector<Vector3f> _vertices, vector<Vector3i> _triangles);
	void InitHEdgeMesh(pPolyMesh polyMesh);

	// Build Half-Edge Structure
	void BuildHalfEdgeMesh();
	void GenerateHalfEdges();
	void ComputeTwinHEdges();
	void UpdateVertices();

	// Compute Neighbors
	void GetNeighborVertices(pHVertex vertex, vector<pHVertex> &out);
	void GetNeighborFaces(pHVertex vertex, vector<pHFace> &out);
	void GetNeighborFaces(pHFace face, vector<pHFace> &out);
	vector<int> GetNeighborFaces(int faceID);

	// Compute Boundary Edges
	void GetBoundaryEdges(vector<int> faceIDs, vector<pHEdge> &outputEdges);
	pHEdge GetSharedEdge(pHFace faceA, pHFace faceB);

	// Utility Functions
	bool IsTwinEdges(pHEdge edgeA, pHEdge edgeB);
	void ComputeVertexNormal(pHVertex vertex);
	vector<pHFace> GetFaceList();

#if USE_OPENGL_DRAW
	// Draw Mesh
	void DrawModel_Poly();
	void DrawModel_Wire();
#endif
};

#endif
