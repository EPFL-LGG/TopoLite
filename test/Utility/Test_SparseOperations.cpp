//
// Created by Robin Jodon on 05.05.20.
//
#include <catch2/catch.hpp>
#include "Utility/SparseOperations.h"


TEST_CASE("stack_col_SparseMat -> Contactenate two sparse matrices A and B horizontally") {

    vector<T> coeff_a, coeff_b;        // list of triplets
    SpMat a(3, 3);
    SpMat b(3, 3);
    SpMat expected(3,6);

    // Create a matrix 
    coeff_a.push_back(Triplet<double>(0, 0, 1));
    coeff_a.push_back(Triplet<double>(1, 2, 1));
    coeff_a.push_back(Triplet<double>(2, 1, 1));
    coeff_a.push_back(Triplet<double>(2, 2, 1));
    coeff_a.push_back(Triplet<double>(2, 0, 1));

    a.setFromTriplets(coeff_a.begin(), coeff_a.end());
    
    // Create b matrix  
    coeff_b.push_back(Triplet<double>(0, 0, 1));
    coeff_b.push_back(Triplet<double>(1, 1, 1));
    coeff_b.push_back(Triplet<double>(2, 2, 1));

    b.setFromTriplets(coeff_b.begin(), coeff_b.end());

    // Add triplets of expected matrix aUb
    coeff_a.push_back(Triplet<double>(2, 5, 1));
    coeff_a.push_back(Triplet<double>(1, 4, 1));
    coeff_a.push_back(Triplet<double>(0, 3, 1));

    // Create the expected result matrix
    expected.setFromTriplets(coeff_a.begin(), coeff_a.end());

    SpMat c(a.rows(), a.cols() + b.cols());
    
    // The function to test
    stack_col_SparseMat(a, b, c);

    // Create a sparseMat from c - expected - Should be close to {0.0}
    SpMat res = (c - expected);
    res.prune(0.0, 1E-7);
    REQUIRE(res.nonZeros() == 0);
}