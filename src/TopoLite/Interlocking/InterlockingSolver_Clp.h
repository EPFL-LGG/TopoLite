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
public:
    InterlockingSolver_Clp(pContactGraph _graph, shared_ptr<InputVarList> varList)
    : InterlockingSolver<Scalar>::InterlockingSolver(_graph, varList)
    {

    }

public:
    bool isTranslationalInterlocking(pInterlockingData data);

    bool isRotationalInterlocking(pInterlockingData data);
};

#include "InterlockingSolver_Clp.cpp"
#endif //TOPOLITE_INTERLOCKINGSOLVER_CLP_H
