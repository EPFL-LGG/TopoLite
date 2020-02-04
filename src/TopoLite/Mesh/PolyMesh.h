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

#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/HelpStruct.h"
#include "TopoLite/Utility/TopoObject.h"
#include "Polygon.h"

using namespace std;
using pPolygon = shared_ptr<_Polygon> ;
using pTriangle = shared_ptr<Triangle> ;

struct duplicate_vertex
{
    Vector3f pos;
    int vID;
    double eps;
};

struct duplicate_vertex_compare
{
    bool operator()(const duplicate_vertex& A, const duplicate_vertex& B) const
    {
        double eps = A.eps / 2;

        if (A.pos[0] - B.pos[0] < -eps)
            return true;
        if (A.pos[0] - B.pos[0] > eps)
            return false;

        if (A.pos[1] - B.pos[1] < -eps)
            return true;
        if (A.pos[1] - B.pos[1] > eps)
            return false;

        if (A.pos[2] - B.pos[2] < -eps)
            return true;
        if (A.pos[2] - B.pos[2] > eps)
            return false;

        return false;
    }
};


class PolyMesh : public TopoObject
{

public:
    //Storage
	vector<Vector3f> vertexList;         // List of vertex positions
	vector<Vector2f> texCoordList;       // List of texture coordinate
	vector<pPolygon> polyList;           // Faces of polygonal mesh
    bool texturedModel;                  // Whether this mesh has texture

public:
	//Computed
	Box bbox;                            // Bounding box of polygonal mesh
	Vector3f centroid;                   // Centroid of polygonal mesh
    float volume;                        // Volume if the mesh is closed
	Vector3f lowestPt;                   // lowest point in terms of y coordinate


public:
    
    PolyMesh(std::shared_ptr<InputVarList> var) : TopoObject(var){}
    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();

	void ClearMesh();
	void Print();
    void NormalizeMesh(Vector3f &trans, float &scale);

    // Read/Save OBJ File
    bool ReadOBJModel(  const char *fileName,
                        bool &textureModel_,
                        bool normalized);
    void WriteOBJModel(const char *objFileName, bool triangulate = false);

    void removeDuplicatedVertices(double eps = FLOAT_ERROR_LARGE);

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
	void ScaleMesh(float scale);
	void RotateMesh(Vector3f rotCenter, Vector3f rotAxis, float rotAngle);

	shared_ptr<PolyMesh> getTextureMesh();

private:

    float ComputeVolume(vector<pTriangle> triList);

};
#endif