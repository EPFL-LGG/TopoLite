//
// Created by robin jodon on 2020-04-16.
//
// Test file to understand how to use CLP
//

#include <catch2/catch.hpp>
#include <iostream>
#include <Eigen/Sparse>

#include <IpIpoptApplication.hpp>
#include "Utility_tests/ipopt_problems.h"


//using namespace Ipopt;


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
TEST_CASE("IPOPT simplex - Simple LP problem") {
    // [1] - Instance of NLP
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
}