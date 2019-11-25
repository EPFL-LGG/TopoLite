//
// Created by ziqwang on 19.02.19.
//

//**************************************************************************************//
//                                   MeshClosenessConstraint
//**************************************************************************************//

#include <libShapeOp/src/Types.h>
#include "CrossConstraint.h"
#include <igl/point_mesh_squared_distance.h>
ShapeOp::MeshClosenessConstraint::MeshClosenessConstraint(const std::vector<int> &idI,
                                                          Scalar weight,
                                                          const Matrix3X &positions) : Constraint(idI, weight)
{
    assert(idI.size() == 1);
}

void ShapeOp::MeshClosenessConstraint::project(const Matrix3X & positions, Matrix3X &projections) const
{
    Eigen::MatrixXd P(1, 3);
    P << positions.col(idI_[0])[0], positions.col(idI_[0])[1], positions.col(idI_[0])[2];
    Eigen::VectorXd sqrD;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;

    aabbTree.lock()->squared_distance(*V, *F, P, sqrD, I, C);
    //igl::point_mesh_squared_distance(P, *V, *F, sqrD, I, C);
    //std::cout << P << "\t" << C << std::endl;
    projections.col(idO_) = weight_ * C.row(0);
}

void ShapeOp::MeshClosenessConstraint::addConstraint(std::vector <Triplet> &triplets, int &idO) const
{
    idO_ = idO;
    triplets.push_back(Triplet(idO_, idI_[0], weight_));
    idO += 1;
}
