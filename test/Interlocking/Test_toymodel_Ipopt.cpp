//
// Created by robin jodon on 2020-04-16.
//
// Test file to understand how to use CLP
//

#include <catch2/catch.hpp>
#include <iostream>

#define HAVE_CSTDDEF
#include <IpIpoptApplication.hpp>
#undef HAVE_CSTDDEF

#include "Problem_LP_simple.h"
#include "Problem_LP_bigm.h"


using namespace Ipopt;

/**
 * @brief Simple test case for Ipopt barrier solver
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
TEST_CASE("IPOPT - Simple LP problem") {
    // [1] - Instance for Ipopt App and NLP
    SmartPtr<problem_LP> mynlp = new problem_LP();
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // [2] - Set some options
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet
    // app->Options()->SetStringValue("derivative_test", "first-order"); // excellent for debugging

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

    REQUIRE(mynlp->x_sol[0] == Approx(2.0).epsilon(1E-7));
    REQUIRE(mynlp->x_sol[1] == Approx(1.0).epsilon(1E-7));
    REQUIRE(mynlp->x_sol[2] == Approx(3.0).epsilon(1E-7));
    printf("Done");
}

/**
 * @brief Simple test case for Ipopt barrier solver
 *        We are solving the following LP problem with the big M method
 *
 *        max 3x + 4y + 2z - sum_{i=1}^{3} \lambda_i M
 *    s.t.    2x            + \lambda_1 >= 4     E1
 *             x      + 2z  + \lambda_2 >= 8     E2
 *                 3y +  z  + \lambda_3 >= 6     E3
 *                (x,y,z,\lambda_i)     >= 0     E4
 *
 *        Solution is: x,y,z = (2,1,3)
 *                     max   = 16
 * 
 * @note Here, we use a different lambda per constraint. 
 *       If lambda is degenerated, it is more flexible for solving, but heavier to compute as well.
 *
 */
TEST_CASE("IPOPT - Simple LP problem with big M - Multidimensional lambda") {
    // [1] - Instance for Ipopt App and NLP
    SmartPtr<problem_LP_bigm> mynlp = new problem_LP_bigm();
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // [2] - Set some options
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet
    // app->Options()->SetStringValue("derivative_test", "first-order"); // excellent for debugging

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

    REQUIRE(mynlp->x_sol[0] == Approx(2.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[1] == Approx(1.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[2] == Approx(3.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[3] == Approx(0.0).epsilon(1E-6));
    REQUIRE(mynlp->x_sol[4] == Approx(0.0).epsilon(1E-6));
    REQUIRE(mynlp->x_sol[5] == Approx(0.0).epsilon(1E-6));

    printf("Done");
}

/**
 * @brief Simple test case for Ipopt barrier solver
 *        We are solving the following LP problem with the big M method
 *
 *        max 3x + 4y + 2z - \lambda M
 *    s.t.    2x            + \lambda >= 4     E1
 *             x      + 2z  + \lambda >= 8     E2
 *                 3y +  z  + \lambda >= 6     E3
 *                (x,y,z,\lambda_i)     >= 0     E4
 *
 *        Solution is: x,y,z = (2,1,3)
 *                     max   = 16
 * 
 */
TEST_CASE("IPOPT - Simple LP problem with big M - Unidimensional lambda") {
    // [1] - Instance for Ipopt App and NLP
    SmartPtr<problem_LP_bigm_unidim> mynlp = new problem_LP_bigm_unidim();
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    // [2] - Set some options
    app->Options()->SetNumericValue("tol", 1e-7);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet
    // app->Options()->SetStringValue("derivative_test", "first-order"); // excellent for debugging

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
    mynlp->x_sol[0];
    REQUIRE(mynlp->x_sol[0] == Approx(2.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[1] == Approx(1.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[2] == Approx(3.0).epsilon(1E-3));
    REQUIRE(mynlp->x_sol[3] == Approx(0.0).epsilon(1E-6));

    printf("Done");
}