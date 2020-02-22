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

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef shared_ptr<VTex<Scalar>> pVTex;

public:

    struct sort_vertex
    {
        Vector3 pos;
        int vID;
        double eps;
        shared_ptr<VPoint<Scalar>> ptr;
    };

    struct sort_vertex_compare
    {
        bool operator()(const sort_vertex& A, const sort_vertex& B) const
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
    vector<pVertex> vertexList;          // Vertex Position of polygonal mesh

    vector<pVTex> texList;               // Tex Coordinate

	Box<Scalar> bbox;                    // Bounding box of polygonal mesh

    Box<Scalar> texBBox;                 // Bounding box of polygonal mesh's texture

	Vector3 centroid;                    // Centroid of polygonal mesh

	Scalar volume;                       // Volume if the mesh is closed

	Vector3 lowestPt;                    // lowest point in terms of y coordinate

public:
    
    PolyMesh(std::shared_ptr<InputVarList> var) : TopoObject(var){}

    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();


/***********************************************
 *             modify mesh operation           *
 ***********************************************/

public:

    void clear();

    void normalize(Vector3 &trans, Scalar &scale);

    void setPolyLists(vector<pPolygon> _polyList);

    // Read OBJ File
    bool readOBJModel(  const char *fileName, bool &textureModel_, bool normalized);

    void removeDuplicatedVertices(double eps = FLOAT_ERROR_LARGE);

    // Transform Mesh
    void translateMesh(Vector3 move);

    void scaleMesh(Vector3 scale);

    void rotateMesh(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle);

    // Auxiliary Data Computation
    void computeBBox();

    void computeTextureBBox();

    void computeCentroid();

    void computeVolume();

    void computeLowestPt();

    void computeVertexList();

    void computeTexList();

public:
/***********************************************
 *             read only mesh operation        *
 ***********************************************/

	void print() const;

    vector<Vector3> getVertices() const{
        vector<Vector3> pointLists;
        for(pVertex vertex: vertexList){
            pointLists.push_back(vertex.pos);
        }
        return pointLists;
    }

    shared_ptr<PolyMesh<Scalar>> getTextureMesh() const;

    //Save OBJ File
    void writeOBJModel(const char *objFileName, bool triangulate = false) const;

    //convert to triangle mesh
    void convertToTriMesh(vector<pTriangle> &triList) const;

private:

    Scalar computeVolume(vector<pTriangle> triList) const;

    Vector3 computeCentroid(vector<pTriangle> triList) const;

    Vector3 computeExtremeVertex(Vector3 rayDir) const;
};

#include "PolyMesh.cpp"

#endif