//
// Created by Robin Jodon on 19.04.20.
//

#include <cassert>
#include <iostream>

#include "ipopt_problems.h"

using namespace Ipopt;


// constructor
problem_NLP::problem_NLP() {}

// destructor
problem_NLP::~problem_NLP() {}

// returns the problem dimensions
bool problem_NLP::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag, IndexStyleEnum &index_style) {
    n = 3;                          // 3 variables
    m = 3;                          // 3 inequalities
    nnz_jac_g = 5;                  // non zero elements in Jacobian
    nnz_h_lag = 0;                  // non-zero elements in Lagrangian Hessian
    index_style = TNLP::C_STYLE;    // use the C style indexing (0-based)

    return true;
}

// returns the variable bounds
bool problem_NLP::get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) {
    // Ensure dimensions are set properly
    assert(n == 3);
    assert(m == 4);

    // lower bounds are 0
    for (Index i = 0; i < n; i++)
        x_l[i] = 0.0;

    // the variables have no upper bounds
    for (Index i = 0; i < n; i++) {
        x_u[i] = COIN_DBL_MAX;
    }

    // constraints
    g_l[0] = -COIN_DBL_MAX;
    g_u[0] = 4;
    g_l[1] = -COIN_DBL_MAX;
    g_u[1] = 8;
    g_l[2] = -COIN_DBL_MAX;
    g_u[2] = 6;

    return true;
}

// returns the initial point for the problem
bool problem_NLP::get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                     Index m, bool init_lambda, Number *lambda) {

    assert(init_x);
    assert(!init_z);
    assert(!init_lambda);

    // initialize to the given starting point
    x[0] = 1.0;
    x[1] = 5.0;
    x[2] = 5.0;

    return true;
}

// return the objective function
bool problem_NLP::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
    obj_value = 3 * x[0] + 4 * x[1] + 2 * x[2]
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool problem_NLP::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
    grad_f[0] = 3
    grad_f[1] = 4
    grad_f[2] = 2
    return true;
}

// return the value of the constraints: g(x)
bool problem_NLP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
    g[0] = 2 * x[0];
    g[1] = x[0] + 2 * x[2];
    g[2] = x[0] + 3 * x[1] + 2 * x[2];
    return true;
}

// return the structure or values of the Jacobian
bool problem_NLP::eval_jac_g(Index n, const Number *x, bool new_x,
                             Index m, Index nele_jac, Index *iRow, Index *jCol, Number *values) {
    if (values == NULL) {
        // return the structure of the Jacobian
        iRow[0] = 0;
        jCol[0] = 0;
        iRow[1] = 0;
        jCol[1] = 1;
        iRow[2] = 0;
        jCol[2] = 2;
        //
        iRow[3] = 1;
        jCol[3] = 0;
        iRow[4] = 1;
        jCol[4] = 1;
        iRow[5] = 1;
        jCol[5] = 2;
        //
        iRow[6] = 2;
        jCol[6] = 0;
        iRow[7] = 2;
        jCol[7] = 1;
        iRow[7] = 2;
        jCol[7] = 2;
    } else {
        // return the values of the Jacobian of the constraints

        values[0] = 2; // 0,0
        values[1] = 0; // 0,1
        values[2] = 0; // 0,2

        values[3] = 1; // 1,0
        values[4] = 0; // 1,1
        values[5] = 2; // 1,2

        values[6] = 0; // 2,0
        values[7] = 3; // 2,1
        values[8] = 1; // 2,2


    }

    return true;
}

//return the structure or values of the Hessian
bool problem_NLP::eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m, const Number *lambda,
                         bool new_lambda, Index nele_hess, Index *iRow, Index *jCol, Number *values) {

    if (values == NULL) {
        // return the structure. This is a symmetric matrix, fill the lower left triangle only.
        Index idx = 0;
        for (Index row = 0; row < 4; row++) {
            for (Index col = 0; col <= row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }
    } else {
        // Hessian is zero
        int hess_dim = m * m;
        for (int id = 0; id < hess_dim; id++)
            values[id] = 0;
    }

    return true;
}

void problem_NLP::finalize_solution(SolverReturn status,
                                    Index n,
                                    const Number *x,
                                    const Number *z_L,
                                    const Number *z_U,
                                    Index m,
                                    const Number *g,
                                    const Number *lambda,
                                    Number obj_value,
                                    const IpoptData *ip_data,
                                    IpoptCalculatedQuantities *ip_cq) {
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // For this example, we write the solution to the console
    printf("Solution of the primal variables, x\n");
    for (Index i = 0; i < n; i++)
        std::cout << "x[" << i << "] = " << x[i] << std::endl;

    printf("Solution of the bound multipliers, z_L and z_U\n");
    for (Index i = 0; i < n; i++)
        std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;

    for (Index i = 0; i < n; i++)
        std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;


    printf("Objective value");
    std::cout << "f(x*) = " << obj_value << std::endl;

    printf("Final values of the constraints\n");
    for (Index i = 0; i < m; i++)
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
}
