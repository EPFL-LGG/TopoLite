//
// Created by Robin Jodon on 05.05.20.
//
#include <catch2/catch_all.hpp>
#include "Utility/SparseOperations.h"

TEST_CASE("create_identity_sparseMat") {
    // Reference identity matrix
    vector<T> triplets; 
    triplets.push_back(Triplet<double>(0, 0, 1));
    triplets.push_back(Triplet<double>(1, 1, 1));
    triplets.push_back(Triplet<double>(2, 2, 1));

    SpMat id(3, 3);
    id.setFromTriplets(triplets.begin(), triplets.end());

    // Call create identity matrix and compare the results
    SpMat r; 
    create_identity_SparseMat(r, 3); 

    SpMat res = r - id ;
    res.prune(0.0, 1E-7);
    REQUIRE(res.nonZeros() == 0);
}


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

    
    // The function to test
    SpMat c;
    stack_col_SparseMat(a, b, c);

    // Create a sparseMat from c - expected - Should be close to {0.0}
    SpMat res = (c - expected);
    res.prune(0.0, 1E-7);
    REQUIRE(res.nonZeros() == 0);
}