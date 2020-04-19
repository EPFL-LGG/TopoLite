//
// Created by robin jodon on 2020-04-16.
//

#include <catch2/catch.hpp>
#include <iostream>
#include <Eigen/Sparse>

#include "Interlocking/InterlockingSolver_Clp.h"
#include "IO/XMLIO.h"
#include "ClpSimplex.hpp"


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
    printf("Definition of the problem\n");
    for (size_t id = 0; id < num_row; id++) {
        constraintLower[id] = -COIN_DBL_MAX;
        constraintUpper[id] = rhs[id];
        objective[id] = -objective_function[id];
        printf("%zu %E %E %E \n", id, constraintLower[id], constraintUpper[id], objective[id]);
    }

    printf("Optimization - start\n");
    ClpSimplex model;
    CoinMessageHandler handler;
    handler.setLogLevel(4);
    model.passInMessageHandler(&handler);
    model.newLanguage(CoinMessages::us_en);
    model.loadProblem(matrix, varLower, varUpper, objective, constraintLower, constraintUpper);
    model.setPrimalTolerance(1e-5);
    model.primal();
    // Get the solution
    const double obj_solution = model.rawObjectiveValue();
    double *solution = model.primalColumnSolution();
    double *row_solution = model.primalRowSolution();

    for (size_t id = 0; id <num_var; id++)
        printf("x[%zu] = %E\n", id+1, solution[id]);
    for (size_t id = 0; id < num_row; id++)
        printf("E[%zu] = %E\n", id+1, row_solution[id]);


    printf("Optimization - done");
}