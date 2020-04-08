//
// Created by ziqwang on 04.04.20.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_CLP_H
#define TOPOLITE_INTERLOCKINGSOLVER_CLP_H

#include "InterlockingSolver.h"
#include "ClpSimplex.hpp"
#include "CoinHelperFunctions.hpp"

template <typename Scalar>
class InterlockingSolver_Clp : public InterlockingSolver<Scalar>{
public:
    typedef shared_ptr<typename InterlockingSolver<Scalar>::InterlockingData> pInterlockingData;
    typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<Scalar>  EigenTriple;
    typedef shared_ptr<ContactGraph<Scalar>> pContactGraph;
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;
    using InterlockingSolver<Scalar>::graph;
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    InterlockingSolver_Clp(pContactGraph _graph, shared_ptr<InputVarList> varList)
    : InterlockingSolver<Scalar>::InterlockingSolver(_graph, varList)
    {

    }

public:
    bool isTranslationalInterlocking(pInterlockingData &data);

    bool isRotationalInterlocking(pInterlockingData &data);

    bool solve(pInterlockingData &data,
            vector<EigenTriple> &tris,
            bool rotationalInterlockingCheck,
            int num_row,
            int num_col,
            int num_var);
};

#include "InterlockingSolver_Clp.cpp"
#endif //TOPOLITE_INTERLOCKINGSOLVER_CLP_H
