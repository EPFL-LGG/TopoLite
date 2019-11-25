///////////////////////////////////////////////////////////////
//
// Mesh.h
//
//   Polygonal Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 09/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _POLY_MESH_H
#define _POLY_MESH_H

#include "Utility/vec.h"
#include "Utility/HelpStruct.h"
#include "Utility/TopoObject.h"
#include "Polygon.h"

using namespace std;
using pPolygon = shared_ptr<_Polygon> ;
using pTriangle = shared_ptr<Triangle> ;

class PolyMesh : public TopoObject
{

public:
    //Storage
	vector<Vector3f> vertexList;         // List of vertex positions
	vector<Vector2f> texCoordList;       // List of texture coordinate
	vector<pPolygon> polyList;           // Faces of polygonal mesh
    bool texturedModel;                  // Whether this mesh has texture

	//Computed
	Box bbox;                            // Bounding box of polygonal mesh
	Vector3f centroid;                   // Centroid of polygonal mesh
    float volume;                        // Volume if the mesh is closed
	Vector3f lowestPt;                   // lowest point in terms of y coordinate


public:

    PolyMesh(){}
    PolyMesh(std::shared_ptr<gluiVarList> var) : TopoObject(var){}
    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();

	void ClearMesh();
	void Print();
    void NormalizeMesh();

    // Read/Save OBJ File
    bool ReadOBJModel(  const char *fileName,
                        bool &textureModel_,
                        bool normalized);
    void WriteOBJModel(const char *objFileName, bool triangulate = false);

    void UpdateVertices();

public:

	// Mesh Operations	
	void ComputeBBox();
	Box ComputeTextureBBox();

	// Volume and Centroid
	void ComputeCentroid();
	Vector3f ComputeCentroid(vector<pTriangle> triList);
	float ComputeVolume();

	void ComputeLowestPt();
    Vector3f ComputeExtremeVertex(Vector3f rayDir);

    // Transform Mesh
	void TranslateMesh(Vector3f move);
	void RotateMesh(Vector3f rotCenter, Vector3f rotAxis, float rotAngle);

private:

    float ComputeVolume(vector<pTriangle> triList);

};
#endif