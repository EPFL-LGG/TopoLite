//
// Created by ziqwang on 2019-12-10.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_H
#define TOPOLITE_INTERLOCKINGSOLVER_H

#include <Eigen/SparseCore>
#include <Eigen/Dense>
#include "ContactGraph.h"

using std::string;
using std::map;
using std::vector;
using std::shared_ptr;

using pairIJ = std::pair<int, int>;
using EigenSpMat = Eigen::SparseMatrix<double>;
using EigenTriple = Eigen::Triplet<double>;
using stdvec_Vector3d = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>;

struct InterlockingData{
    stdvec_Vector3d traslation;
    stdvec_Vector3d rotation;
    vector<int> partID;
};

struct EquilibriumData{
    stdvec_Vector3d force;
    stdvec_Vector3d torque;
    stdvec_Vector3d contact_points;
    vector<pairIJ> partIJ;
};

class InterlockingSolver{
public:
    vector<shared_ptr<ContactGraphNode>> nodes;
    vector<shared_ptr<ContactGraphEdge>> edges;
    //automatic generate
    vector<weak_ptr<ContactGraphNode>> dynamic_nodes;
protected:

    /*************************************************
    *      Building block for the Interlocking Matrix
    *************************************************/

    void get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                  Eigen::RowVector2d    &fkx, Eigen::RowVector2d &fky, Eigen::RowVector2d &fkz);

    void get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                  Eigen::RowVector4d    &fkx, Eigen::RowVector4d &fky, Eigen::RowVector4d &fkz);

    void get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
                                          Eigen::RowVector2d &mx, Eigen::RowVector2d &my, Eigen::RowVector2d &mz);

    void get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
                                          Eigen::RowVector4d &mx, Eigen::RowVector4d &my, Eigen::RowVector4d &mz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction = false);

public:

    /*************************************************
    *           Compute Interlocking Matrix
    *************************************************/

    void computeEquilibriumMatrix(Eigen::MatrixXd &mat, bool withFriction = false);

    double computeEquilibriumMatrixConditonalNumber();

    void computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

public:

    /*************************************************
    *           Interlocking Test
    *************************************************/

    virtual bool isTranslationalInterlocking(shared_ptr<InterlockingData> data){ return  true;}

    virtual bool isRotationalInterlocking(shared_ptr<InterlockingData> data){ return  true;}

    virtual bool isEquilibrium(EigenPoint gravity, shared_ptr<EquilibriumData> data){ return  true;}
};


#endif //TOPOLITE_INTERLOCKINGSOLVER_H
