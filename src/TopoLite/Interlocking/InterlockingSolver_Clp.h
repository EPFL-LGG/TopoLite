//
// Created by ziqwang on 04.04.20.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_CLP_H
#define TOPOLITE_INTERLOCKINGSOLVER_CLP_H

#include "InterlockingSolver.h"
#include "ClpSimplex.hpp"
#include "CoinHelperFunctions.hpp"

enum CLP_SOLVER_TYPE{
    SIMPLEX,
    BARRIER
};

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

    CLP_SOLVER_TYPE type;

public:
    InterlockingSolver_Clp(pContactGraph _graph,
            shared_ptr<InputVarList> varList,
            CLP_SOLVER_TYPE _type = SIMPLEX): InterlockingSolver<Scalar>::InterlockingSolver(_graph, varList), type(_type)
    {

    }

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

    bool solveSimplex(pInterlockingData &data,
               bool rotationalInterlockingCheck,
               int num_row,
               int num_col,
               int num_var,
               const CoinPackedMatrix& matrix,
               const double *colLower,
               const double *colUpper,
               const double *objective,
               const double *rowLower,
               const double *rowUpper);

    bool solveBarrier(pInterlockingData &data,
                      bool rotationalInterlockingCheck,
                      int num_row,
                      int num_col,
                      int num_var,
                      const CoinPackedMatrix& matrix,
                      const double *colLower,
                      const double *colUpper,
                      const double *objective,
                      const double *rowLower,
                      const double *rowUpper);

    void unpackSolution(InterlockingSolver_Clp::pInterlockingData& data, bool rotationalInterlockingCheck, const double *solution, int num_var);
};

#endif //TOPOLITE_INTERLOCKINGSOLVER_CLP_H
