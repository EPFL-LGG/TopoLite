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

#include "Utility/vec.h"
#include "Utility/TopoObject.h"

#include "Mesh/HEdgeMesh.h"
#include "Mesh/Cross.h"
#include "PartGeom.h"


using namespace std;

class Part;
using wpPart = weak_ptr<Part>;
using wpCross =  weak_ptr<Cross> ;

using pPart =  shared_ptr<Part> ;
using pPolyMesh =  shared_ptr<PolyMesh> ;
using pCross =  shared_ptr<Cross>;
using pPolygon = shared_ptr<_Polygon> ;
using pPartGeom = shared_ptr<PartGeom>;

class Part : public TopoObject
{
public:

    int partID;                             // Part ID in the structure
	int assemblyID;
	int groupID;                            // Part group that the part falls in

	wpCross cross;                          // Base polygon used to construct the part geometry
	pPolyMesh polyMesh;                     // Resulted polyhedron
	pPartGeom partGeom;                     // Generate part geometry

public:
    //

	bool atBoundary;                        // If part at boundary of the structure
    bool isRemove;                          // If part is removed from the structure (i.e., disassembly)
    bool touchGround;                       // If part touches the ground

	vector<wpPart>  initNeighbors;           // Neighboring parts (saved in the same order as oriPoints)

public:
    //rendering parameters

	Vector3f mtlDiffuse;                    // Rendering material diffuse
	Vector3f mtlSpecular;                   // Rendering material specular

	Vector3f text3DPos;                     // Position to draw 3D text;
	Vector3f textLinkPt;                    // Position to draw linking point to the part
	Vector3f wireColor;

public:

	Part(pCross cross, shared_ptr<InputVarList> var);
	Part(const Part &part);
	~Part();
	void Clear();
	void PrintPart();

public:
    //output
	void WriteOBJModel(char *objFileName);
    void WriteOBJModel(char *objFileName, Vector3f movement);
	void WriteOBJWireFrameModel(const char *objFileName);

public:
	// Compute Part Geometry
	bool CheckLegalGeometry(bool use_orient_opt = false);
	void ComputePartGeometry(bool convexPart, Vector2f cutPlaneHeight, bool previewMode);
	void Compute3DTextPosition();
	Vector2f computeYExtrem();

    // Part Operations
	int GetNeighborIndex(pPart neiborPart);
	int GetFaceIndex    (pPolygon neiborPartFace);
	void GetContactFaces(pPart neiborPart, vector<pair<int, int>> &contaFaceIDPairs);
};

#endif