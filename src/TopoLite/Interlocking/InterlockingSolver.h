//
// Created by ziqwang on 2019-12-10.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_H
#define TOPOLITE_INTERLOCKINGSOLVER_H
#include "Eigen/Sparse"
#include <Eigen/SparseCore>
#include <Eigen/Dense>
#include "ContactGraph.h"
#include "Utility/TopoObject.h"

using std::string;
using std::map;
using std::vector;
using std::shared_ptr;


template<typename Scalar>
class InterlockingSolver: public TopoObject{
public:

    typedef Matrix<double, 3, 1> Vector3;
    typedef Matrix<double, 1, 2> RowVector2;
    typedef Matrix<double, 1, 4> RowVector4;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<double>  EigenTriple;
    typedef std::vector<Vector3,Eigen::aligned_allocator<Vector3>> stdvec_Vector3d;
    typedef shared_ptr<VPoint<Scalar>> pVertex;

public:

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

public:

    shared_ptr<ContactGraph<Scalar>> graph;

public:

    InterlockingSolver(shared_ptr<ContactGraph<Scalar>> _graph, shared_ptr<InputVarList> varList):TopoObject(varList)
    {
        graph = _graph;
    }


protected:

    /*************************************************
    *      Building block for the Interlocking Matrix
    *************************************************/

    void get_force_from_norm_fric(Vector3    n,    Vector3   u,  Vector3   v,
                                  RowVector2    &fkx, RowVector2 &fky, RowVector2 &fkz);

    void get_force_from_norm_fric(Vector3    n,    Vector3   u,  Vector3   v,
                                  RowVector4    &fkx, RowVector4 &fky, RowVector4 &fkz);

    void get_moment_from_norm_fric_vertex(Vector3 n,   Vector3 u,   Vector3 v, Vector3 r,
                                          RowVector2 &mx, RowVector2 &my, RowVector2 &mz);

    void get_moment_from_norm_fric_vertex(Vector3 n,   Vector3 u,   Vector3 v, Vector3 r,
                                          RowVector4 &mx, RowVector4 &my, RowVector4 &mz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction = false);

public:

    /*************************************************
    *           Compute Interlocking Matrix
    *************************************************/

    void computeEquilibriumMatrix(Eigen::MatrixXd &mat, bool withFriction = false);

    Scalar computeEquilibriumMatrixConditonalNumber();

    void computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrixDense(Eigen::MatrixXd &mat);

    void computeTranslationalInterlockingMatrixDense(Eigen::MatrixXd &mat);

    void computeRotationalInterlockingMatrixSparse(EigenSpMat &mat);

public:

    /*************************************************
    *           Interlocking Test
    *************************************************/

    virtual bool isTranslationalInterlocking(shared_ptr<InterlockingData> data){ return  true;}

    virtual bool isRotationalInterlocking(shared_ptr<InterlockingData> data){ return  true;}

    virtual bool isEquilibrium(Vector3 gravity, shared_ptr<EquilibriumData> data){ return  true;}
};

#include "InterlockingSolver.cpp"

#endif //TOPOLITE_INTERLOCKINGSOLVER_H
