//
// Created by robin jodon on 2020-04-16.
//
// Test file to understand how to use CLP
//

#include <catch2/catch.hpp>
#include <iostream>
#include <Eigen/Sparse>

#include "Interlocking/InterlockingSolver_Clp.h"
#include "IO/JsonIOReader.h"
#include "ClpSimplex.hpp"


typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

/**
 * @brief Simple test case of CLP simplex solver
 *        We are solving the following LP problem
 *
 *        max 3x + 4y + 2z
 *    s.t.    2x           >= 4     E1
 *             x      + 2z >= 8     E2
 *                 3y +  z >= 6     E3
 *             (x,y,z)     >= 0     E4
 *
 *        Solution is: x,y,z = (2,1,3)
 *                     max   = 16
 */
TEST_CASE("CLP simplex - Simple LP problem") {
    // Mock
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVar(varList.get());
    shared_ptr<ContactGraph<double>> dat = make_shared<ContactGraph<double>>(varList);
    InterlockingSolver_Clp<double> solver(dat, varList);
    shared_ptr<typename InterlockingSolver<double>::InterlockingData> data;
    //
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
    coef.emplace_back(T(1, 0, 1));
    coef.emplace_back(T(1, 2, 2));
    coef.emplace_back(T(2, 1, 3));
    coef.emplace_back(T(2, 2, 1));

    A.setFromTriplets(coef.begin(), coef.end());

    CoinPackedMatrix matrix(true, num_row, num_col, A.nonZeros(), A.valuePtr(), A.innerIndexPtr(), A.outerIndexPtr(), A.innerNonZeroPtr());

    // boundaries for rows and columns values

    auto *objective(new double[num_var]);           // objective function params
    auto *constraintLower(new double[num_row]);     // E1...E3 lower bounds
    auto *constraintUpper(new double[num_row]);     // E1...E3 upper bounds
    auto *varLower(new double[num_var]);            // E4
    auto *varUpper(new double[num_var]);            // E4

    // E4
    for (size_t id = 0; id < num_var; id++) {
        varLower[id] = 0;
        varUpper[id] = COIN_DBL_MAX;
    }

    // Objective func + Ei for i in [1,2,3]
    for (size_t id = 0; id < num_row; id++) {
        constraintLower[id] = -COIN_DBL_MAX;
        constraintUpper[id] = rhs[id];
        objective[id] = -objective_function[id];  // simplex minimises by default
    }

    ClpSimplex model;
    CoinMessageHandler handler;
    handler.setLogLevel(0);
    model.passInMessageHandler(&handler);
    model.newLanguage(CoinMessages::us_en);
    model.loadProblem(matrix, varLower, varUpper, objective, constraintLower, constraintUpper);
    model.setPrimalTolerance(1e-9);
    model.primal();
    // Get the solution
    const double obj_solution = model.rawObjectiveValue();
    double *solution = model.primalColumnSolution();
    double *row_solution = model.primalRowSolution();

    SECTION("CLP simplex - Solutions checkup") {
        REQUIRE(solution[0] == Approx(2.0).epsilon(1E-9));
        REQUIRE(solution[1] == Approx(1.0).epsilon(1E-9));
        REQUIRE(solution[2] == Approx(3.0).epsilon(1E-9));
    }

    SECTION("CLP simplex - Constraints checkup") {
        REQUIRE(row_solution[0] == Approx(4.0).epsilon(1E-9));
        REQUIRE(row_solution[1] == Approx(8.0).epsilon(1E-9));
        REQUIRE(row_solution[2] == Approx(6.0).epsilon(1E-9));
    }

    SECTION("CLP simplex - Problem checkup") {
        REQUIRE(model.isProvenOptimal());
        REQUIRE_FALSE(model.isProvenPrimalInfeasible());
    }
}