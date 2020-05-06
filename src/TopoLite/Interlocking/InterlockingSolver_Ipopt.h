//
// Created by robinjodon on 29.04.20.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_IPOPT_H
#define TOPOLITE_INTERLOCKINGSOLVER_IPOPT_H

#include "InterlockingSolver.h"
#include "CoinHelperFunctions.hpp"


template<typename Scalar>
class InterlockingSolver_Ipopt : public InterlockingSolver<Scalar> {
public:
    typedef shared_ptr<typename InterlockingSolver<Scalar>::InterlockingData> pInterlockingData;
    typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor> EigenSpMat;
    typedef Eigen::Triplet<Scalar> EigenTriple;
    typedef shared_ptr<ContactGraph<Scalar>> pContactGraph;
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;
    using InterlockingSolver<Scalar>::graph;
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    InterlockingSolver_Ipopt(pContactGraph _graph, shared_ptr<InputVarList> varList) :
     InterlockingSolver<Scalar>::InterlockingSolver(_graph, varList) {}

public:

    bool isTranslationalInterlocking(pInterlockingData &data);

    bool isRotationalInterlocking(pInterlockingData &data);

    bool checkSpecialCase(pInterlockingData &data,
                          vector<EigenTriple> copy_tris,
                          bool rotationalInterlockingCheck,
                          Eigen::Vector2i copy_size);

    bool solve(pInterlockingData &data,
               vector<EigenTriple> &tris,
               bool rotationalInterlockingCheck,
               int num_row,
               int num_col,
               int num_var);

    void unpackSolution(InterlockingSolver_Ipopt::pInterlockingData &data, 
                        bool rotationalInterlockingCheck, 
                        const double *solution,
                        int num_var);
};

#include "InterlockingSolver_Ipopt.cpp"

#endif //TOPOLITE_INTERLOCKINGSOLVER_IPOPT_H
