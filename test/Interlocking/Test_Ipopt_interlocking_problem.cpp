#include <catch2/catch.hpp>
#include <iostream>
#include "Interlocking/InterlockingSolver_Ipopt.h"
#include <Eigen/Sparse> 

using namespace Eigen;


TEST_CASE("eval_g") {
    SmartPtr<IpoptProblem> ipb = new IpoptProblem();       // problem to solve

    std::vector<Triplet<double>> triplets; 
    triplets.push_back(Triplet<double>(0, 0, 1));
    triplets.push_back(Triplet<double>(0, 1, 1));
    triplets.push_back(Triplet<double>(2, 1, 1));
    triplets.push_back(Triplet<double>(3, 5, 1));

    SparseMatrix<double> b(4, 6);
    b.setFromTriplets(triplets.begin(), triplets.end());
    ipb->b_coeff = b;

    VectorXd g(6), x(6), g_expected(4);
    x << 1, 2, 3, 4, 5, 6; 
    g_expected << 3, 0, 2, 6;
    g.setZero();

    const double *x_ptr;
    double *g_ptr;

    x_ptr = &x(0); 
    g_ptr = &g(0);

    ipb->eval_g(6, x_ptr, false, 4, g_ptr);
    for (int i = 0; i < 4; i++)
        REQUIRE(g_ptr[i] == g_expected[i]); 
}

TEST_CASE("eval_jac_g") {
    SmartPtr<IpoptProblem> ipb = new IpoptProblem();       // problem to solve

    std::vector<Triplet<double>> triplets; 
    triplets.push_back(Triplet<double>(0, 0, 1));
    triplets.push_back(Triplet<double>(0, 1, 2));
    triplets.push_back(Triplet<double>(2, 1, 1));
    triplets.push_back(Triplet<double>(3, 5, 1));

    SparseMatrix<double> b(4, 6);
    b.setFromTriplets(triplets.begin(), triplets.end());
    ipb->b_coeff = b;

    VectorXd j(4), j_expected(4);
    j_expected << 1, 2, 1, 1;
    j.setZero();

    VectorXi row(4), col(4);
    row.setZero();
    col.setZero();

    int *iRow;
    int *iCol;
    iRow = &row(0);
    iCol = &col(0); 

    double *j_ptr;
    j_ptr = &j(0);

    SECTION("Check Jacobian values") {
        ipb->eval_jac_g(0, 0, false, 0, 4, iRow, iCol, j_ptr);

        for (int i = 0; i < 4; i++)
            REQUIRE(j_ptr[i] == j_expected[i]); 
    }

    SECTION("Check Jacobian triplets indexes") {
        ipb->eval_jac_g(0, 0, false, 0, 4, iRow, iCol, nullptr);

        for (int i = 0; i < 4; i++) {
            REQUIRE(iRow[i] == triplets[i].row()); 
            REQUIRE(iCol[i] == triplets[i].col()); 
        }
    }
}
