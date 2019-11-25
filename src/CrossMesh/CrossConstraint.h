//
// Created by ziqwang on 19.02.19.
//

#ifndef TOPOLOCKCREATOR_CROSSCONSTRAINT_H
#define TOPOLOCKCREATOR_CROSSCONSTRAINT_H

#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>
#include <libShapeOp/src/Common.h>
#include <igl/AABB.h>
#include <memory>
namespace ShapeOp
{
class SHAPEOP_API MeshClosenessConstraint : public Constraint
{
public:

    MeshClosenessConstraint(const std::vector<int> &idI, Scalar weight, const Matrix3X &positions);
    virtual ~MeshClosenessConstraint() {}
    /** \brief Find the closest configuration from the input positions that satisfy the constraint.*/
    virtual void project(const Matrix3X & /*positions*/, Matrix3X &projections) const override final;
    /** \brief Add the constraint to the linear system.*/
    virtual void addConstraint(std::vector<Triplet> &triplets, int &idO) const override final;

public:

    void setTree(std::weak_ptr<igl::AABB<Eigen::MatrixXd, 3>> _aabbTree,
                 std::shared_ptr<Eigen::MatrixXd> _V,
                 std::shared_ptr<Eigen::MatrixXi> _F)
    {
        aabbTree = _aabbTree;
        V = _V;
        F = _F;
    }

public:
    int id0;
    std::weak_ptr<igl::AABB<Eigen::MatrixXd, 3>> aabbTree;
    std::shared_ptr<Eigen::MatrixXd> V;
    std::shared_ptr<Eigen::MatrixXi> F;
};
}


#endif //TOPOLOCKCREATOR_CROSSCONSTRAINT_H
