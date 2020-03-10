//
// Created by ziqwang on 09.01.19.
//

#ifndef TOPOLOCKCREATOR_QUADTREE_H
#define TOPOLOCKCREATOR_QUADTREE_H

#include <Eigen/Dense>
#include "Mesh/PolyMesh.h"
#include <igl/AABB.h>

using Eigen::Vector2d;

/*!
 * \brief 2D Kd-Tree, for quickly searching the triangle where given points are located
 */

template<typename Scalar>
class PolyMesh_AABBTree : public PolyMesh<Scalar>
{
public:
    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;
    typedef Matrix<Scalar, 2, 1> Vector2;
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

public:

    igl::AABB<MatrixX, 2> texTree;
    igl::AABB<MatrixX, 3> posTree;

    MatrixX pV, tV;
    MatrixXi pF, tF;

    Eigen::VectorXi tC, pC;

public:

    PolyMesh_AABBTree(const PolyMesh<Scalar> &_mesh)
    :PolyMesh<Scalar>::PolyMesh(_mesh.getVarList())
    {
        PolyMesh<Scalar>::polyList = _mesh.polyList;
        PolyMesh<Scalar>::vertexList = _mesh.vertexList;
        PolyMesh<Scalar>::textureList = _mesh.textureList;
        PolyMesh<Scalar>::texturedModel = _mesh.texturedModel;
    }

public:

    void buildTexTree()
    {
        PolyMesh<Scalar>::convertTexToEigenMesh(tV, tF, tC);
        texTree.init(tV, tF);
    }

    void buildPosTree(){
        PolyMesh<Scalar>::convertPosToEigenMesh(pV, pF, pC);
        posTree.init(pV, pF);
    }

public:

    pPolygon findTexPoint(VectorX pt)
    {
        vector<int> indices = texTree.find(tV, tF, pt.transpose(), false);
        if(!indices.empty()){
            int faceID = tC(indices[0]);
            return PolyMesh<Scalar>::polyList[faceID];
        }
        return nullptr;
    }

    Vector3d findMeshNearestPoint(Vector3 pt);

};


#endif //TOPOLOCKCREATOR_QUADTREE_H
