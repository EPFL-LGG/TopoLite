//
// Created by Robin Jodon on 19.04.20.
//

#include <cassert>
#include <iostream>

#include "Problem_LP_simple.h"

using namespace Ipopt;

const double COIN_DBL_MAX = std::numeric_limits<double>::max();


// constructor
problem_LP::problem_LP() {
    x_sol = {0,0,0};            // initial position vector that becomes the solution
    hessian_triplets_nb = 0;
};

// destructor
problem_LP::~problem_LP() = default;

// returns the problem dimensions
bool problem_LP::get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag, IndexStyleEnum &index_style) {
    n = 3;                          // 3 variables
    m = 3;                          // 3 inequalities
    nnz_jac_g = 5;                  // non zero elements in Jacobian
    nnz_h_lag = 0;                  // non-zero elements in Lagrangian Hessian
    index_style = TNLP::C_STYLE;    // use the C style indexing (0-based)

    return true;
}

// returns the variable bounds
bool problem_LP::get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) {
    // Ensure dimensions are set properly
    assert(n == 3);
    assert(m == 3);

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
bool problem_LP::get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                     Index m, bool init_lambda, Number *lambda) {

    assert(init_x);
    assert(!init_z);
    assert(!init_lambda);

    // initialize to the given starting point
    x[0] = x_sol[0];
    x[1] = x_sol[1];
    x[2] = x_sol[2];

    return true;
}

// return the objective function
bool problem_LP::eval_f(Index n, const Number *x, bool new_x, Number &obj_value) {
    // minus sign everywhere, we are looking for max
    obj_value = 3 * x[0] + 4 * x[1] + 2 * x[2];
    obj_value = -1.0 * obj_value;
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool problem_LP::eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) {
    // minus sign everywhere, we are looking for max
    grad_f[0] = -3;
    grad_f[1] = -4;
    grad_f[2] = -2;
    return true;
}

// return the value of the constraints: g(x)
bool problem_LP::eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) {
    g[0] = 2 * x[0];
    g[1] = 1 * x[0] + 2 * x[2];
    g[2] = 3 * x[1] + 1 * x[2];
    return true;
}

// return the triplet structure or values of the Jacobian
bool problem_LP::eval_jac_g(Index n, const Number *x, bool new_x,
                             Index m, Index nele_jac, Index *iRow, Index *jCol, Number *values) {
    if (values == nullptr) {
        // return the 2 first triplet index row, col for the structure of the Jacobian
        iRow[0] = 0; jCol[0] = 0;
        iRow[1] = 1; jCol[1] = 0;
        iRow[2] = 1; jCol[2] = 2;
        iRow[3] = 2; jCol[3] = 1;
        iRow[4] = 2; jCol[4] = 2;
    } else {
        // return the values of the Jacobian of the constraints

        values[0] = 2.0; // 0,0
        values[1] = 1.0; // 1,0
        values[2] = 2.0; // 1,2
        values[3] = 3.0; // 2,1
        values[4] = 1.0; // 2,2
    }
    return true;
}

//return the structure or values of the Hessian
bool problem_LP::eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m, const Number *lambda,
                         bool new_lambda, Index nele_hess, Index *iRow, Index *jCol, Number *values) {

    if (values == nullptr) {
        // return the structure. This is a symmetric matrix, fill the lower left triangle only.
        Index idx = 0;
        for (Index row = 0; row < hessian_triplets_nb; row++) {
            for (Index col = 0; col <= row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }
    } else {
        // Hessian is zero
        for (int id = 0; id < hessian_triplets_nb; id++)
            values[id] = 0;
    }

    return true;
}

void problem_LP::finalize_solution(SolverReturn status,
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


    // For this example, we write the solution to the console
    printf("Solution of the primal variables, x\n");
    for (Index i = 0; i < n; i++) {
        this->x_sol[i] = x[i];
        printf("--  x[%d] = %E\n", i, x[i]);
    }

    printf("Solution of the bound multipliers, z_L and z_U\n");
    for (Index i = 0; i < n; i++)
        printf("--  z_L[%d] = %E\n", i, z_L[i]);

    for (Index i = 0; i < n; i++)
        printf("--  z_U[%d] = %E\n", i, z_U[i]);

    printf("Objective value");
    printf("-- f(x*) = %E\n", obj_value);

    printf("Final values of the constraints\n");
    for (Index i = 0; i < m; i++)
        printf("--  g[%d] = %E\n", i, g[i]);
}
