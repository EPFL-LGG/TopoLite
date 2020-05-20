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

#include "Utility/GeometricPrimitives.h"
#include "Mesh/Cross.h"
#include "Mesh/PolyMesh.h"

template<typename Scalar>
class ConvexBlock : public TopoObject
{
public:

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef weak_ptr<Cross<Scalar>> wpCross;
    typedef shared_ptr<Cross<Scalar>> pCross;
    typedef shared_ptr<HypPlane<Scalar>> pHypPlane;
    typedef shared_ptr<HypVertex<Scalar>> pHypVertex;
    typedef Matrix<Scalar, 2, 1> Vector2;
    typedef Matrix<Scalar, 3, 1> Vector3;


public:

	wpCross cross;                          // Base polygon used to construct the part geometry
	pPolyMesh polyMesh;                     // Resulted polyhedron
    Vector2 cutter_heights;
    int partID;
    bool atBoundary;

private:

    vector<pHypPlane> hypList;
    vector<pHypVertex> corners;

public:

    ConvexBlock(pCross cross, Vector2 _cutter_heights);
    ConvexBlock(const ConvexBlock &part);
	~ConvexBlock();
	void clear();
	void print();

public:

    //
    void computeCorners();

    //
    void computeHyperPlanes();

    //
    void computeFaces();

    // Compute Part Geometry
    bool checkGeometry();

public:

	virtual bool compute();
};

#endif