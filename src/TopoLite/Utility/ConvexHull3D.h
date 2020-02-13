//
// Created by ziqwang on 31.03.19.
//

#ifndef TOPOLOCKCREATOR_CONVEXHULL3D_H
#define TOPOLOCKCREATOR_CONVEXHULL3D_H

#include <Eigen/Dense>
using Eigen::Matrix;
using Eigen::Dynamic;
template<typename Scalar>
class ConvexHull3D
{

public:
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef std::vector<Matrix<Scalar, 3, 1>> ListVector3;
    typedef std::vector<Matrix<int, 3, 1>> ListVector3i;

public:
    void computeQuickHull(ListVector3 &pointList, ListVector3 &ver, ListVector3i &tri);
    void computeQuickHull(ListVector3 &pointList, Matrix<Scalar, Dynamic, Dynamic> &V, Matrix<int, Dynamic, Dynamic> &F);

};

#include "ConvexHull3D.cpp"

#endif //TOPOLOCKCREATOR_CONVEXHULL3D_H
