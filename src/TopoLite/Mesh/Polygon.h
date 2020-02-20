///////////////////////////////////////////////////////////////
//
// Mesh/Polygon.h
//
//   3D Polygon (vertices may or may not be co-planar)
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef _POLYGON_H
#define _POLYGON_H

#include "TopoLite/Utility/GeometricPrimitives.h"
#include <Eigen/Dense>
#include <vector>

using namespace std;
using Eigen::Matrix;

/*!
 * @brief: Polygon class
 * @note: we do not name the class as Polygon to avoid confusion with Polygon defined in windows.h
 */
template<typename Scalar>
class _Polygon
{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, 2, 1> Vector2;
    typedef Matrix<int, 3, 1> Vector3i;
    typedef shared_ptr<Triangle<Scalar>> pTriangle;
    typedef shared_ptr<_Vertex<Scalar>> pVertex;

public:

    //storage
	vector<pVertex> vers;        //!< Vertices   (position, texture coordinate.)
	vector<int> verIDs;                  //!< Vertex IDs (only for position)
	vector<int> texIDs;					 //!< texture ID

    //computed
	Vector3 normal;                     //!< Normal vector
    Vector3 center;                     //!< Center point

	//auxiliary
    int dist;                            //!< Discrete distance to a root polygon (e.g., BFS search)
    int polyType;                        //!< Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation

public:

    //Basic Operations

    _Polygon();
    _Polygon(const _Polygon &poly);
    _Polygon & operator=(const _Polygon &poly);
	~_Polygon();

    void setVertices(vector<Vector3> _vers);

    void reverseVertices();

    bool checkEquality(const _Polygon &poly);

    //Mimic the vector class

    virtual void print();

    void clear(){
        vers.clear();
        verIDs.clear();
        texIDs.clear();
        normal = center = Vector3(0, 0, 0);
        dist = polyType = 0;
    }

    size_t push_back(Vector3 pt);

    size_t size() const{return vers.size();}

    Vector3 operator [](int index) const{
        if(vers.empty()) return Vector3(0, 0, 0);
        else{
            int rID = index % size();
            return vers.at(rID).pos;
        }
    }

    vector<Vector3> getVertices()
    {
        vector<Vector3> vertices;
        for (int i = 0; i < vers.size(); i++)
            vertices.push_back( vers[i].pos );
        return vertices;
    }

    vector<Vector2> getVerticesTex()
    {
        vector<Vector2> vertices;
        for (int i = 0; i < vers.size(); i++)
        {
            vertices.push_back( vers[i].texCoord );
        }
        return vertices;
    }

    void setDistance(int _dist){dist = _dist;}

    int getDistance(){return dist;}

    void setPolyType(float _polyType){polyType = _polyType;}

    int getPolyType(){return polyType;}

public:

    // Compute Properties
	Vector3 computeCenter();

	Vector3 computeNormal();

	Vector3 computeFitedPlaneNormal();

	Scalar computeArea();

    Scalar computeAverageEdge();

    Scalar computeMaxRadius();

    void computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin);

public:

    // Polygon Operations
    
	void convertToTriangles(vector<pTriangle> &tris);

	int getPtVerID(Vector3 point);

public:

	// Transforms
	
	void executeTranslation(Vector3 transVec);

//	vector<Vector3> ProjectPolygonTo2D(double projMat[]);
//
//	void ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[]);
//
//	vector<Vector3i> ProjectToNormalPlane(Vector3 x_axis, Vector3 y_axis, Vector3 origin, float Scale);
};

#include "Polygon.cpp"

#endif