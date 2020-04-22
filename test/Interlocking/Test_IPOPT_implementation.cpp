//
// Created by robin jodon on 2020-04-16.
//
// Test file to understand how to use CLP
//

#include <catch2/catch.hpp>
#include <iostream>
#include <Eigen/Sparse>

#include <IpIpoptApplication.hpp>
#include "ipopt_problems.h"


using namespace Ipopt;


typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

int ipoptSolver() {
    // [1] - Instance for Ipopt App and NLP
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    SmartPtr<TNLP> mynlp = new problem_NLP();

    // [2] - Set some options
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    //app->Options()->SetStringValue("mehrota_algorithm", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet
    app->Options()->SetStringValue("derivative_test", "first-order");


    // [3] - Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        printf("\n\n*** Error during initialization!\n");
        return status;
    }
    // [4] - Optimzation
    status = app->OptimizeTNLP(mynlp);
    if (status == Solve_Succeeded) {
        printf("\n\n*** The problem solved!\n");
    } else {
        printf("\n\n*** The problem FAILED!\n");
    }
// As the SmartPtrs go out of scope, the reference count // will be decremented and the objects will automatically // be deleted.
    return status;
}

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
TEST_CASE("IPOPT simplex - Simple LP problem") {
    // [1] - Instance for Ipopt App and NLP
    SmartPtr<TNLP> mynlp = new problem_NLP();
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // [2] - Set some options
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    //app->Options()->SetStringValue("mehrota_algorithm", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet
    app->Options()->SetStringValue("derivative_test", "first-order");


    // [3] - Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        printf("\n\n*** Error during initialization!\n");
        REQUIRE(0 == 1);
    }
    // [4] - Optimzation
    status = app->OptimizeTNLP(mynlp);
    if (status == Solve_Succeeded) {
        printf("\n\n*** The problem solved!\n");
    } else {
        printf("\n\n*** The problem FAILED!\n");
        REQUIRE(0 == 1);
    }
    printf("yo");
}