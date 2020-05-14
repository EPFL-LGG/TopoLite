//
// Created by Robin Jodon on 05.05.20.
//

#ifndef TOPOLITE_SPARSEOPERATIONS_H
#define TOPOLITE_SPARSEOPERATIONS_H

#include <Eigen/Sparse>
#include <iostream>

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
using namespace std;
using namespace Eigen;

/**
 * @brief Create a identity SparseMat object
 * 
 * @tparam Scalar 
 * @param m the matrix the create 
 * @param dim the dimensions of the matrix. 
 */
template<typename Scalar>
void create_identity_SparseMat(SparseMatrix<Scalar>  &m, int dim){
    std::vector<Triplet<double>> triplets;
    m.resize(dim, dim);
    for (int i=0; i<dim; i++)
        triplets.push_back(Triplet<double>(i,i,1));

    m.setFromTriplets(triplets.begin(), triplets.end());
}

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
void stack_col_SparseMat(const SparseMatrix<Scalar> &B, const SparseMatrix<Scalar>  &I, SparseMatrix<Scalar>  &C){
    C.resize(B.rows(), B.cols() + I.cols());
    C.reserve(B.nonZeros() + I.cols());
    for (int c = 0; c < B.rows(); ++c) {
        for (SparseMatrix<double>::InnerIterator it(B, c); it; ++it) {
            C.insert(it.row(), it.col()) = it.value();
        }
        for (SparseMatrix<double>::InnerIterator it(I, c); it; ++it) {
            C.insert(it.row(), (B.cols() + it.col())) = it.value();
        }
    }
    C.finalize();
}

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
inline void stack_row_SparseMat(SpMat &B, SpMat &I, SpMat &C){

}

/**
 * @brief Print out the triplets row, col, value
 *  
 * @param A the sparse matrix to print
 */
template<typename Scalar>
void print_SparseMatTriplets(const SparseMatrix<Scalar> &A){
    for (int i = 0; i < A.rows(); ++i) {
        for (SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
            // fixme: the template will break if <Scalar> is not <double>
            printf("%ld %ld %f\n", it.row(), it.col(), it.value());
        }
    }
    printf("\n");
}


/**
 * @brief Print out as a dense matrix
 *  
 * @param A the sparse matrix to print
 */
template<typename Scalar>
void print_SparseMatTriplets(const SparseMatrix<Scalar> &A, const int precision=0){
    std::cout.precision(precision);
    std::cout << Eigen::MatrixXd(A).format(StreamPrecision) << std::endl;
}

#endif //TOPOLITE_SPARSEOPERATIONS_H
