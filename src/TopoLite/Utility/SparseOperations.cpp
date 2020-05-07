//
// Created by Robin Jodon on 05.05.20.
//
#include <Eigen/Sparse>

typedef Eigen::SparseMatrix<double> SpMat; // declares a column-major sparse matrix type of double
typedef Eigen::Triplet<double> T;
using namespace std;
using namespace Eigen;



template<typename Scalar>
void create_identity_SparseMat(SparseMatrix<Scalar>  &m, int dim) {
    std::vector<Triplet<double>> triplets;
    m.resize(dim, dim);
    for (int i=0; i<dim; i++)
        triplets.push_back(Triplet<double>(i,i,1));

    m.setFromTriplets(triplets.begin(), triplets.end());
}


template<typename Scalar>
void stack_col_SparseMat(const SparseMatrix<Scalar> &B, const SparseMatrix<Scalar>  &I, SparseMatrix<Scalar>  &C) {
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

void stack_row_SparseMat(SpMat &B, SpMat &I, SpMat &C) {
    // Not implemented
}

template<typename Scalar>
void print_SparseMatTriplets(const SparseMatrix<Scalar> &A) { 
    for (int i = 0; i < A.rows(); ++i) {
        for (SparseMatrix<double>::InnerIterator it(A, i); it; ++it) {
            // fixme: the template will break if <Scalar> is not <double>
            printf("%ld %ld %f\n", it.row(), it.col(), it.value());
        }
    }
    printf("\n");
}

template<typename Scalar>
void print_SparseMat(const SparseMatrix<Scalar> &A, const int precision=0) { 
    std::cout.precision(precision);
    std::cout << Eigen::MatrixXd(A).format(StreamPrecision) << std::endl;
}