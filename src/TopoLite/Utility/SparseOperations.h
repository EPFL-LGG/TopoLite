//
// Created by Robin Jodon on 05.05.20.
//

#ifndef TOPOLITE_SPARSEOPERATIONS_H
#define TOPOLITE_SPARSEOPERATIONS_H
#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
using namespace std;
using namespace Eigen;

/**
 * @brief Concatenate B and I horizontally
 *
 *          / B00 B01 B02 | I00 I01 \
 *     C =  | B10 B11 B12 | I10 I11 |
 *          \ B20 B21 B22 | I20 I21 /
 *
 * @tparam Scalar
 * @param B Left sparse matrix
 * @param I Right sparse matrix
 * @param C Final sparse matrix
 */

template<typename Scalar>
void stack_col_SparseMat(const SparseMatrix<Scalar> &B, const SparseMatrix<Scalar>  &I, SparseMatrix<Scalar>  &C);

/**
 * @brief Concatenate B and I vertically
 *
 *          / B00 B01 B02 \
 *          | B10 B11 B12 |
 *      C =  -------------
 *          | I00 I01 I02 |
 *          \ I10 I11 I12 /
 *
 * @tparam Scalar
 * @param B Left sparse matrix
 * @param I Right sparse matrix
 * @param C Final sparse matrix
 */
void stack_row_SparseMat(SpMat &B, SpMat &I, SpMat &C);

/**
 * @brief Print out the triplets row, col, value
 *  
 * @param A the sparse matrix to print
 */
template<typename Scalar>
void print_SparseMat(const SparseMatrix<Scalar> &A);

#include "SparseOperations.cpp"

#endif //TOPOLITE_SPARSEOPERATIONS_H
