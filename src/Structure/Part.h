///////////////////////////////////////////////////////////////
//
// Part.h
//
//   Part Model Class
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 15/July/2018
//
///////////////////////////////////////////////////////////////


#ifndef _PART_H
#define _PART_H

#include <vector>

#include "Mesh/HEdgeMesh.h"
#include "Utility/Controls.h"
#include "Utility/vec.h"

using namespace std;

class Cross;
struct OrientPoint;
class PartGeom;
class BodyMobili;
class _Polygon;
class PolyMesh;

class Part;
typedef weak_ptr<Part> wpPart;
typedef shared_ptr<Part> pPart;
typedef shared_ptr<PolyMesh> pPolyMesh;
typedef shared_ptr<Cross> pCross;
typedef weak_ptr<Cross> wpCross;
typedef shared_ptr<_Polygon> pPolygon;

typedef shared_ptr<PartGeom> pPartGeom;
typedef shared_ptr<BodyMobili> pBodyMobili;

class Part 
{
public:

    int partID;                             // Part ID in the structure
	int assemblyID;
	int groupID;                            // Part group that the part falls in

	wpCross cross;                          // Base polygon used to construct the part geometry
	shared_ptr<PolyMesh> polyMesh;          // Resulted polyhedron
	pPartGeom partGeom;                     // Generate part geometry

public:
	bool atBoundary;                        // If part at boundary of the structure
    bool isRemove;                          // If part is removed from the structure (i.e., disassembly)
    bool touchGround;                       // If part touches the ground

	pBodyMobili partMobili;                 // Generate part mobility space (for debug)
	vector<wpPart>  initNeighbors;           // Neighboring parts (saved in the same order as oriPoints)

public:
	Vector3f mtlDiffuse;                    // Rendering material diffuse
	Vector3f mtlSpecular;                   // Rendering material specular

	Vector3f text3DPos;                     // Position to draw 3D text;
	Vector3f textLinkPt;                    // Position to draw linking point to the part
	Vector3f wire_color;

public:
	Part(pCross cross);
	Part(const Part &part);
	~Part();
	void Clear();
	void PrintPart();
	void WriteOBJModel(char *objFileName);
    void WriteOBJModel(char *objFileName, Vector3f movement);
	void WriteOBJWireFrameModel(const char *objFileName);

	// Compute Part Geometry
	bool CheckLegalGeometry(bool use_orient_opt = false);
	void ComputePartGeometry(bool convexPart, Vector2f cutPlaneHeight, bool previewMode);
	void Compute3DTextPosition();
	Vector2f computeYExtrem();

	// Compute Part Mobility
	void EvaluateBodyMobility();
	Vector3f GetPartMoveDirection();

    // Part Operations
	int GetNeighborIndex(pPart neiborPart);
	int GetFaceIndex(pPolygon neiborPartFace);
	void GetContactFaces(pPart neiborPart, vector<pair<int, int>> &contaFaceIDPairs);

#if USE_OPENGL_DRAW
public:

	// Draw Part
	void DrawPart();
	void DrawPartWire(float width, Vector3f color);
	void DrawPart3DText(float textScale, float width, Vector3f color);
	void DrawPartBBox(float width, Vector3f color);
	void DrawPartCentroid(float size, Vector3f color);
	void DrawPartNormals();

	// Draw Part Interaction
	// void DrawContactPolys();
	void DrawContactForces();
	void DrawSupport();

	// Draw Part Construction
	void DrawOriPoints();
	void DrawInnerPolygon();
	void DrawGeomFaces();
	void DrawGeomEdges();
	void DrawGeomVertices();

	// Draw Part Mobility
	void DrawMobiliFaces();
	void DrawMobiliEdges();
	void DrawMobiliVertices();
	void DrawMobiliRays();
	void DrawMobiliVector();
	void DrawMobiliMesh();
#endif

public: //useless code(for this project)
	vector<int> contactIDs;
	vector<vector<float>> contactForces;
	vector<vector<Vector3f>> contactVertices;
	vector<double> supportForces;
	vector<wpPart>  currNeighbors;           // Neighboring parts (saved in the same order as oriPoints)
	vector<vector<wpPart>> frozen_sets;
};

#endif