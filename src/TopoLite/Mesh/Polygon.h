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

private:
    //storage
	vector<pVertex> vers_;        //!< Vertices Position

	vector<pVTex> texs_;          //!< Tex coordinates

	int dist_;                            //!< Discrete distance to a root polygon (e.g., BFS search)

	int polyType_;                        //!< Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation

    //computed
	Vector3 normal_;                     //!< Normal vector

    Vector3 center_;                     //!< Center point

    bool compute_data_dirty;            //!< dirty means need to recompute normal and center;

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

    //will make computed data dirty

    void setVertices(vector<Vector3> _vers);

    size_t push_back(Vector3 pt);

    size_t push_back(Vector3 pt, Vector2 tex);

    pVertex useless_ver;
    pVertex& vers(int index)
    {
        if(vers_.empty()) return useless_ver;

        dirty();
        index = index % vers_.size();
        return vers_.at(index);
    }

    pVTex useless_tex;
    pVTex& texs(int index)
    {
        if(texs_.empty()) return useless_tex;

        dirty();
        index = index % texs_.size();
        return texs_.at(index);
    }

public:

    //update and get computed data

    void update()
    {
        normal_ = computeNormal();
        center_ = computeCenter();
        compute_data_dirty = false;
    }

    void dirty(){compute_data_dirty = true;}

    Vector3 normal()
    {
        if(compute_data_dirty) update();
        return normal_;
    }

    Vector3 center(){
        if(compute_data_dirty) update();
            compute_data_dirty = false;
        return center_;
    }

public:

    //won't make computed data dirty

    void reverseVertices();

    void setDistance(int _dist){dist_ = _dist;}

    void setPolyType(float _polyType){polyType_ = _polyType;}

    void translatePolygon(Vector3 transVec);

    void clear()
    {
        vers_.clear();
        normal_ = center_ = Vector3(0, 0, 0);
        dist_ = polyType_ = 0;
        compute_data_dirty = false;
    }

/***********************************************
 *                                             *
 *             read only      operation        *
 *                                             *
 ***********************************************/

public:

    bool checkEquality(const _Polygon &poly) const;

    virtual void print() const;

    size_t size() const {return vers_.size();}

    vector<Vector3> getVertices() const
    {
        vector<Vector3> vertices;
        for (int i = 0; i < vers_.size(); i++)
            vertices.push_back( vers_[i]->pos );
        return vertices;
    }

    vector<Vector2> getVerticesTex() const
    {
        vector<Vector2> vertices;
        for (int i = 0; i < vers_.size(); i++)
        {
            vertices.push_back( vers_[i].texCoord );
        }
        return vertices;
    }

    int getDistance() const {return dist_;}

    int getPolyType() const {return polyType_;}

public:

    //compute the data

    Vector3 computeCenter() const;

    Vector3 computeNormal() const;

    Vector3 computeFitedPlaneNormal() const;

    Scalar computeArea() const;

    Scalar computeAverageEdge() const;

    Scalar computeMaxRadius() const;

    void computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const;

public:

    // Polygon Operations
	void convertToTriangles(vector<pTriangle> &tris) const;

	int getPtVerID(Vector3 point) const;

};

#include "Polygon.cpp"

#endif