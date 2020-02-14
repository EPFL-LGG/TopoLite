///////////////////////////////////////////////////////////////
//
// ConvexHull_2D.h
//
//   Compute 2D Convex Hull for a set of co-planar points
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 10/Nov/2017
//
//
///////////////////////////////////////////////////////////////

#ifndef CONVEXHULL2D_H
#define CONVEXHULL2D_H

#include <Eigen/Dense>
#include <vector>

using Eigen::Matrix;

template<typename Scalar>
class ConvexHull2D{

public:
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef std::vector<Matrix<Scalar, 3, 1>> ListVector3;

    ConvexHull2D(){

    }

public:

    // Chain Hull Algorithm
    void compute(const ListVector3 &in, ListVector3 &out);

private:

    Scalar cross(const Vector3 &O, const Vector3 &A, const Vector3 &B);

    // Sort Points Based on X - and Y - coordinate
    void sortXY(ListVector3 &Array);
};

#include "ConvexHull2D.cpp"

#endif