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
#include "TopoLite/Utility/PolyPolyBoolean.h"
#include "Polygon.h"
#include <Eigen/Dense>

using Eigen::Matrix;

template<typename Scalar>
class PolyMesh : public TopoObject
{
public:

    using pPolygon = shared_ptr<_Polygon<Scalar>> ;

    using wpPolygon = weak_ptr<_Polygon<Scalar>> ;

    using pTriangle = shared_ptr<Triangle<Scalar>> ;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef shared_ptr<VTex<Scalar>> pVTex;

    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

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

    vector<pVTex> textureList;               // Texture Coordinate

public:
    
    PolyMesh(std::shared_ptr<InputVarList> var) : TopoObject(var){
        clear();
    }

    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();


/***********************************************
 *             modify mesh operation           *
 ***********************************************/

public:

    void clear();

    void update(){
        removeDuplicatedVertices();
        computeTextureList();
    }

    std::pair<Matrix<Scalar, 3, 1>, Scalar> normalize();

    void setPolyLists(vector<pPolygon> _polyList);

    // Read OBJ File
    bool readOBJModel(  const char *fileName, bool &textureModel_, bool normalized);

    void removeDuplicatedVertices(double eps = FLOAT_ERROR_LARGE);

    void mergeFaces(double eps = 1e-3);

    // Update vertexList and textureList

    void computeVertexList();

    void computeTextureList();

    // Transform Mesh
    void translateMesh(Vector3 move);

    void scaleMesh(Vector3 scale);

    void rotateMesh(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle);

public:
/***********************************************
 *             read only mesh operation        *
 ***********************************************/

	void print() const;

    vector<Vector3> getVertices() const{
        vector<Vector3> pointLists;
        for(pVertex vertex: vertexList){
            pointLists.push_back(vertex->pos);
        }
        return pointLists;
    }

    shared_ptr<PolyMesh<Scalar>> getTextureMesh() const;

    //Save OBJ File
    void writeOBJModel(const char *objFileName, bool triangulate = false) const;

    //convert to triangle mesh
    void convertToTriMesh(vector<pTriangle> &triList) const;

    //convert to Eigen mesh
    void convertPosToEigenMesh(MatrixX &V, MatrixXi &F, Eigen::VectorXi &C);

    void convertTexToEigenMesh(MatrixX &V, MatrixXi &F, Eigen::VectorXi &C);

    void convertPosTexToEigenMesh(MatrixX &V, MatrixX &T, MatrixXi &F);

    // Auxiliary Data Computation
    Box<Scalar> bbox() const;

    Box<Scalar> texBBox() const;

    Vector3 centroid() const;

    Scalar volume() const;

    Vector3 lowestPt() const;

    size_t size(){return polyList.size();}

private:

    Scalar computeVolume(vector<pTriangle> triList) const;

    Vector3 computeCentroid(vector<pTriangle> triList) const;

    Vector3 computeExtremeVertex(Vector3 rayDir) const;
};
#endif