//
// Created by Robin Jodon on 05.05.20.
//
#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
using namespace std;
using namespace Eigen;

template<typename Scalar>
void stack_col_SparseMat(const SparseMatrix<Scalar> &B, const SparseMatrix<Scalar>  &I, SparseMatrix<Scalar>  &C) {
    C.reserve(B.nonZeros() + I.nonZeros());
    for (Index c = 0; c < B.outerSize(); ++c) {
        for (SparseMatrix<double>::InnerIterator it(B, c); it; ++it) {
            C.insert(it.row(), it.col()) = it.value();
        }
        for (SparseMatrix<double>::InnerIterator it(I, c); it; ++it) {
            C.insert(it.row(), it.col() + B.cols()) = it.value();
        }
    }
    C.finalize();
}

void stack_row_SparseMat(SpMat &B, SpMat &I, SpMat &C) {
    // Not implemented
}

template<typename Scalar>
void print_SparseMat(const SparseMatrix<Scalar> &A) { 
    for (Index i = 0; i < A.outerSize(); ++i) {
        for (SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
            // fixme: the template will break if <Scalar> is not <double>
            printf("%ld %ld %f\n", it.row(), it.col(), it.value());
        }
    }
    printf("\n");
}
//void stack_row_SparseMat(SpMat &B, SpMat &I, SpMat &C) {}
