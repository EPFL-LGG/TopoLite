//
// Created by ziqwang on 2020-02-03.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H
#define TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H

#include "InterlockingSolver.h"
#include "Eigen/SparseQR"

template <typename Scalar>
class InterlockingSolver_AffineScaling : public  InterlockingSolver<Scalar>{
public:
    typedef shared_ptr<typename InterlockingSolver<Scalar>::InterlockingData> pInterlockingData;
    typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<Scalar>  EigenTriple;
public:
    InterlockingSolver_AffineScaling(shared_ptr<ContactGraph<Scalar>> _graph, shared_ptr<InputVarList> varList)
    : InterlockingSolver<Scalar>::InterlockingSolver(_graph, varList)
    {

    }

public:
    bool isTranslationalInterlocking(pInterlockingData data);

    bool isRotationalInterlocking(pInterlockingData data);
};

#include "InterlockingSolver_AffineScaling.cpp"

#endif //TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H
