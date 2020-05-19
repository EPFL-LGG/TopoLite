//
// Created by ziqwang on 2019-11-25.
//

#ifndef TOPOLITE_BASEMESHOPTIMIZER_H
#define TOPOLITE_BASEMESHOPTIMIZER_H

#include "Utility/TopoObject.h"
#include "Mesh/CrossMesh.h"
#include <igl/AABB.h>
#include <Eigen/Dense>


class BaseMeshOptimizer : public TopoObject{

public:

    weak_ptr<igl::AABB<Eigen::MatrixXd,3>> aabbTree;

    std::shared_ptr<Eigen::MatrixXd> aabbV;

    std::shared_ptr<Eigen::MatrixXi> aabbF;

public:

    BaseMeshOptimizer() : TopoObject(var){
        aabbV = V;
        aabbF = F;
        aabbTree = _aabbTree;
    }

    void OptimizeBaseMesh(shared_ptr<CrossMesh> crossMesh);

};


#endif //TOPOLITE_BASEMESHOPTIMIZER_H
