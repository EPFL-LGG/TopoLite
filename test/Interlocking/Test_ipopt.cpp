//
// Created by robin jodon on 2020-04-16.
//

#include <catch2/catch.hpp>
#include <iostream>
#include <Eigen/Sparse>

#include "Interlocking/InterlockingSolver_Clp.h"
#include "IO/XMLIO.h"

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

TEST_CASE("Simple case - SolveSimplex") {
    // Mock
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    shared_ptr<ContactGraph<double>> dat = make_shared<ContactGraph<double>>(varList);
    InterlockingSolver_Clp<double> solver(dat, varList);
    shared_ptr<typename InterlockingSolver<double>::InterlockingData> data;

    int num_col = 3;
    int num_row = 3;
    int num_var = 3;

    Vector3d objective_function(3, 4, 2);   // coefficients in objective function
    Vector3d rhs(4, 8, 6);                  // the right hand side vector resulting from the constraints
    SpMat A(num_row, num_col);                      // coefficient matrix

    // Define A
    std::vector<T> coef;
    //
    coef.emplace_back(T(0, 0, 2));
    coef.emplace_back(T(0, 1, 1));
    coef.emplace_back(T(2, 1, 2));
    coef.emplace_back(T(1, 2, 3));
    coef.emplace_back(T(2, 2, 1));

    A.setFromTriplets(coef.begin(), coef.end());

    CoinPackedMatrix matrix(true, num_row, num_col, A.nonZeros(), A.valuePtr(), A.innerIndexPtr(), A.outerIndexPtr(), A.innerNonZeroPtr());

    // boundaries for rows and columns values

    auto *objective(new double[num_var]);
    auto *constraintLower(new double[num_row]);
    auto *constraintUpper(new double[num_row]);
    auto *varLower(new double[num_var]);
    auto *varUpper(new double[num_var]);

    for (size_t id = 0; id < num_var; id++) {
        varLower[id] = 0;
        varUpper[id] = COIN_DBL_MAX;
    }

    for (size_t id = 0; id < num_row; id++) {
        constraintLower[id] = -COIN_DBL_MAX;
        constraintUpper[id] = rhs[id];
        objective[id] = objective_function[id];
//        cout << id << " " << rhs[id] << endl;
    }

    solver.solveSimplex(data, false, num_row, num_col, num_var, matrix,
                        varLower, varUpper,
                        objective,
                        constraintLower, constraintUpper);

    printf("Hello");
}