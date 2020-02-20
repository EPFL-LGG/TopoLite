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

#include "TopoLite/Utility/GeometricPrimitives.h"
#include "TopoLite/Utility/TopoObject.h"
#include "Polygon.h"
#include <Eigen/Dense>
using Eigen::Matrix;

template<typename Scalar>
class PolyMesh : public TopoObject
{
public:
    using pPolygon = shared_ptr<_Polygon<Scalar>> ;
    using pTriangle = shared_ptr<Triangle<Scalar>> ;
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, 2, 1> Vector2;

public:

    struct duplicate_vertex
    {
        Vector3 pos;
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

public:

    //Storage
	vector<pPolygon> polyList;           // Faces of polygonal mesh
    bool texturedModel;                  // Whether this mesh has texture

public:

	//Computed
	vector<_Vertex<Scalar>> vertexList;
	Box<Scalar> bbox;                    // Bounding box of polygonal mesh
	Vector3 centroid;                    // Centroid of polygonal mesh
    Scalar volume;                       // Volume if the mesh is closed
	Vector3 lowestPt;                    // lowest point in terms of y coordinate

public:
    
    PolyMesh(std::shared_ptr<InputVarList> var) : TopoObject(var){}
    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();

public:

	void clear();

	void print();

    void normalize(Vector3 &trans, Scalar &scale);

public:

    //Read OBJ File
    bool readOBJModel(  const char *fileName, bool &textureModel_, bool normalized);

    //Save OBJ File
    void writeOBJModel(const char *objFileName, bool triangulate = false);

    void removeDuplicatedVertices(double eps = FLOAT_ERROR_LARGE);

    shared_ptr<PolyMesh<Scalar>> getTextureMesh();

public:

	// Mesh Operations	
	void computeBBox();
	Box<Scalar> computeTextureBBox();

	// Volume and Centroid
	void computeCentroid();
	Scalar computeVolume();

	void computeLowestPt();
    Vector3 computeExtremeVertex(Vector3 rayDir);

public:

    // Transform Mesh
	void translateMesh(Vector3 move);

	void scaleMesh(Scalar scale);

	void rotateMesh(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle);

private:

    Scalar ComputeVolume(vector<pTriangle> triList);

    Vector3 computeCentroid(vector<pTriangle> triList);

};

#include "PolyMesh.cpp"

#endif