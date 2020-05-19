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

enum PolygonType{
    POLY_NONE = 0,
    POLY_SQUARE_THETA_45        =   1,
    POLY_SQUARE_THETA_15        =   2,
    POLY_SQUARE_THETA_75        =   3,
    POLY_RHOMBUS_THETA_0        =   4,
    POLY_RHOMBUS_THETA_90       =   5,
    POLY_RHOMBUS_THETA_120      =   6,
    POLY_RHOMBUS_THETA_240      =   7,
    POLY_HEXAGON_TYPE_0         =   8,
    POLY_HEXAGON_TYPE_1         =   9,
    POLY_OCTAGON_REGULAR        =   10,
    POLY_OCTAGON_COLINEAR       =   11,
    POLY_DODECAGON              =   12,
    POLY_PENTAGON_CROSS_TYPE_0  =   13,
    POLY_PENTAGON_CROSS_TYPE_1  =   14,
    POLY_PENTAGON_CROSS_TYPE_2  =   15,
    POLY_PENTAGON_CROSS_TYPE_3  =   16,
    POLY_PENTAGON_SNOW_TYPE_0   =   17,
    POLY_PENTAGON_SNOW_TYPE_1   =   18,
    POLY_PENTAGON_SNOW_TYPE_2   =   19,
    POLY_PENTAGON_SNOW_TYPE_3   =   20,
    POLY_PENTAGON_SNOW_TYPE_4   =   21,
    POLY_PENTAGON_SNOW_TYPE_5   =   22,
    POLY_PENTAGON_MIRROR_TYPE_0 =   23,
    POLY_PENTAGON_MIRROR_TYPE_1 =   24,
    POLY_RHOMBUS_TYPE_0         =   25,
    POLY_RHOMBUS_TYPE_1         =   26
};

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
	vector<pVertex> vers;        // Vertices Position

	vector<pVTex> texs;          // Tex coordinates

    //boundary edge
    vector<bool> edge_at_boundary;

private:

    PolygonType polyType;                        // Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation

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

    void setPolyType(int _polyType){
        polyType = static_cast<PolygonType >(_polyType);
    }

    void setPolyType(PolygonType _polyType){
        polyType = _polyType;
    }


    void translatePolygon(Vector3 transVec);

    void translatePolygonTex(Vector2 transVec);

    void clear()
    {
        vers.clear();
        texs.clear();
        polyType = POLY_NONE;
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

    bool at_boundary(int edge_index) const{
        if(edge_index >= 0 && edge_index < edge_at_boundary.size()){
            return edge_at_boundary[edge_index];
        }
        //also consider it as a boundary edge if there is no edge_at_boundary variables
        return true;
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

    int getPolyType() const {
        return static_cast<int>(polyType);
    }

public:

    //compute the data

    Vector3 center() const;

    Vector3 normal() const;

    Vector3 computeFitedPlaneNormal() const;

    Scalar area() const;

    Scalar average_edge() const;

    Scalar max_radius() const;

    void computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const;

    //the find the barycentric coordinates of the tex polygon
    vector<Scalar> computeBaryCentric(Vector2 pt) const;

    vector<Scalar> computeBaryCentric(Vector2 sta, Vector2 end, Vector2 pt) const;

public:

    // Polygon Operations
	void triangulateNaive(vector<pTriangle> &tris) const;

    void triangulate(vector<pTriangle> &tris) const;

	int getPtVerID(Vector3 point) const;

};
#endif
