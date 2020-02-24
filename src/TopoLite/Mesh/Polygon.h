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
using std::make_shared;
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
    typedef shared_ptr<VPoint<Scalar>> pVertex;
    typedef shared_ptr<VTex<Scalar>> pVTex;

public:
    //storage
	vector<pVertex> vers;        //!< Vertices Position

	vector<pVTex> texs;          //!< Tex coordinates

private:
	int dist;                            //!< Discrete distance to a root polygon (e.g., BFS search)

	int polyType;                        //!< Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation

public:

/***********************************************
 *                                             *
 *            Construct and Destroy            *
 *                                             *
 ***********************************************/

    _Polygon();

    _Polygon(const _Polygon &poly);

    ~_Polygon();

/***********************************************
 *                                             *
 *             modify      operation           *
 *                                             *
 ***********************************************/

public:

    void setVertices(vector<Vector3> _vers);

    size_t push_back(Vector3 pt);

    size_t push_back(Vector3 pt, Vector2 tex);

    void reverseVertices();

    void setDistance(int _dist){dist = _dist;}

    void setPolyType(float _polyType){polyType = _polyType;}

    void translatePolygon(Vector3 transVec);

    void clear()
    {
        vers.clear();
        texs.clear();
        dist = polyType = 0;
    }

/***********************************************
 *                                             *
 *             read only      operation        *
 *                                             *
 ***********************************************/

public:

    Vector3 pos(int index)
    {
        if(vers.empty()) return Vector3(0, 0, 0);
        index = index % vers.size();
        return vers.at(index)->pos;
    }

    Vector2 tex(int index){
        if(texs.empty()) return Vector2(0, 0);
        index = index % texs.size();
        return texs.at(index)->texCoord;
    }

    bool checkEquality(const _Polygon &poly) const;

    virtual void print() const;

    size_t size() const {return vers.size();}

    vector<Vector3> getVertices() const
    {
        vector<Vector3> vertices;
        for (size_t i = 0; i < vers.size(); i++)
            vertices.push_back( vers[i]->pos );
        return vertices;
    }

    vector<Vector2> getVerticesTex() const
    {
        vector<Vector2> texCoords;
        for (size_t i = 0; i < texs.size(); i++)
        {
            texCoords.push_back( texs[i]->texCoord);
        }
        return texCoords;
    }

    int getDistance() const {return dist;}

    int getPolyType() const {return polyType;}

public:

    //compute the data

    Vector3 center() const;

    Vector3 normal() const;

    Vector3 computeFitedPlaneNormal() const;

    Scalar area() const;

    Scalar average_edge() const;

    Scalar max_radius() const;

    void computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const;

public:

    // Polygon Operations
	void convertToTriangles(vector<pTriangle> &tris) const;

	int getPtVerID(Vector3 point) const;

};

#include "Polygon.cpp"

#endif