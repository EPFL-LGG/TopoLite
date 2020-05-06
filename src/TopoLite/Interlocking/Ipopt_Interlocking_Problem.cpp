//
// Created by Robin Jodon on 29.04.20.
//

#include <cassert>
#include <iostream>

#include "Ipopt_Interlocking_Problem.h"
#include "Utility/SparseOperations.h"

using namespace Ipopt;

const double COIN_DBL_MAX = std::numeric_limits<double>::max();


// constructor
IpoptProblem::IpoptProblem() {
    n_var = 0;                                          // nb of variables [x, t] + lambda
    n_var_real = 0;                                     // nf of variables [x, t] 
    n_constraints = 0;                              
    non_zero_jacobian_elements = 0; 
    non_zero_hessian_elements = 0;
    index_style = TNLP::C_STYLE;
    obj_value = 0;
}

// destructor
IpoptProblem::~IpoptProblem() = default;


// returns the problem dimensions
bool IpoptProblem::get_nlp_info(int &n, int &m, int &nnz_jac_g, int &nnz_h_lag, IndexStyleEnum &index_style) {
    n = this->n_var;                                    // N+M variables [x, t, lamdba]
    m = this->n_constraints;                            // M inequalities
    nnz_jac_g = this->non_zero_jacobian_elements;       // non zero elements in Jacobian
    nnz_h_lag = this->non_zero_hessian_elements;        // non-zero elements in Lagrangian Hessian
    index_style = this->index_style;                    // use the C style indexing (0-based)

    return true;
}

bool IpoptProblem::initialize(int n, int m, EigenSpMat &mat) {
    this->n_var_real = n;                                   // variables without big M multipliers lambdas
    this->n_var = n + m;                                    // variables
    this->n_constraints = m;                                // Constraints inequalities
    this->non_zero_jacobian_elements = mat.nonZeros() + m;  // non zero elements in Jacobian (initial B matrix + id(m,m))
    this->non_zero_hessian_elements = 0;                    // non-zero elements in Lagrangian Hessian

    set_bounds_info();

    compute_constraints_matrix(mat);

    return true;
}

void IpoptProblem::compute_constraints_matrix(EigenSpMat &mat) {
    EigenSpMat lambda_mat; 
    create_identity_SparseMat(lambda_mat, this->n_constraints);
    stack_col_SparseMat(mat, lambda_mat, this->b_coeff);
}

// returns the variable bounds
bool IpoptProblem::get_bounds_info(int n, Number *x_l, Number *x_u, int m, Number *g_l, Number *g_u) {

    for (int i = 0; i < this->n_var; i++) {
        x_l[i] = this->x_l[i];
        x_u[i] = this->x_u[i];
    }

    for (int i = 0; i < this->n_constraints; i++) {
        g_l[i] = this->g_l[i];
        g_u[i] = this->g_u[i];
    }

    return true;
}

bool IpoptProblem::set_bounds_info() {
    for (int i = 0; i < this->n_constraints; i++) {
        this->g_l[i] = 0.0;
        this->g_u[i] = 1.0;
    }

    for (int i = 0; i < this->n_var; i++) {
        if (i < this->n_var_real) {
            // x_i is defined in R
            this->x_l[i] = COIN_DBL_MAX * (-1.0);
            this->x_u[i] = COIN_DBL_MAX;
        } else {
            // lambda_i >= 0
            this->x_l[i] = 0.0;
            this->x_u[i] = COIN_DBL_MAX;
        }
    }
    return true;
}

// returns the initial point for the problem
bool IpoptProblem::get_starting_point(int n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                      int m, bool init_lambda, Number *lambda) {

    for (int i = 0; i < this->n_var; i++) {
        this->x[i] = 0.0;
        x[i] = 0.0;
    }
    return true;
}

// return the objective function
bool IpoptProblem::eval_f(int n, const Number *x, bool new_x, Number &obj_value) {
    this->obj_value = 0.0;
    double r;
    // fixme: the big M matrix is here
    for (int i = 0; i < this->n_var; i++) {
        if (i < this->n_var_real)
            this->obj_value += 0;
        else
            this->obj_value += x[i] * this->big_m + 1.0;
    }
    this->obj_value = -1.0 * this->obj_value; // minus sign for searching for the maximum
    obj_value = this->obj_value;
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool IpoptProblem::eval_grad_f(int n, const Number *x, bool new_x, Number *grad_f) {
    // fixme: find a neater/faster construction
    for (int i = 0; i < this->n_var; i++)
        if (i < this->n_var_real) {
            grad_f[i] = 0;
        } else {
            grad_f[i] = this->big_m;
        }
    return true;
}

// return the value of the constraints: g(x)
bool IpoptProblem::eval_g(int n, const Number *x, bool new_x, int m, Number *g) {
    // g = B * X
    for (int k = 0; k < this->b_coeff.outerSize(); ++k) {
        for (SparseMatrix<double>::InnerIterator it(this->b_coeff, k); it; ++it) {
            g[it.row()] += it.value() * x[it.col()];
        }
    }
    return true;
}

// return the triplet structure or values of the Jacobian
bool IpoptProblem::eval_jac_g(int n, const Number *x, bool new_x,
                              int m, int nele_jac, int *iRow, int *jCol, Number *values) {
    if (values == nullptr) {
        // return the 2 first triplet index row, col for the structure of the Jacobian
        int idx = 0;
        for (int k = 0; k < this->b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(this->b_coeff, k); it; ++it) {
                iRow[idx] = it.row();
                jCol[idx] = it.col();
                idx++;
            }
        }
    } else {
        // loop through B sparse elements and get its values.
        int idx = 0;
        for (int k = 0; k < this->b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(this->b_coeff, k); it; ++it) {
                values[idx] = it.value();
            }
        }

    }
    return true;
}

//return the structure or values of the Hessian
bool IpoptProblem::eval_h(int n, const Number *x, bool new_x, Number obj_factor, int m, const Number *lambda,
                          bool new_lambda, int nele_hess, int *iRow, int *jCol, Number *values) {

    if (values == nullptr) {
        // return the structure. This is a symmetric matrix, fill the lower left triangle only.
        int idx = 0;
        for (int row = 0; row < this->non_zero_hessian_elements; row++) {
            for (int col = 0; col <= row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }
    } else {
        // Hessian is zero
        // fixme: faster and neater declaration here
        int hess_dim = (this->n_constraints + this->n_var) * this->n_constraints;
        for (int id = 0; id < hess_dim; id++)
            values[id] = 0;
    }

    return true;
}

void IpoptProblem::finalize_solution(SolverReturn status,
                                     int n,
                                     const Number *x,
                                     const Number *z_L,
                                     const Number *z_U,
                                     int m,
                                     const Number *g,
                                     const Number *lambda,
                                     Number obj_value,
                                     const IpoptData *ip_data,
                                     IpoptCalculatedQuantities *ip_cq) {


    // For this example, we write the solution to the console
    printf("Solution of the primal variables, x\n");
    for (int i = 0; i < n; i++) {
        this->x_solution[i] = x[i];
        printf("--  x[%d] = %E\n", i, x[i]);
    }

    printf("Solution of the bound multipliers, z_L and z_U\n");
    for (int i = 0; i < n; i++)
        printf("--  z_L[%d] = %E\n", i, z_L[i]);

    for (int i = 0; i < n; i++)
        printf("--  z_U[%d] = %E\n", i, z_U[i]);

    printf("Objective value");
    printf("-- f(x*) = %E\n", obj_value);

    printf("Final values of the constraints\n");
    for (int i = 0; i < m; i++)
        printf("--  g[%d] = %E\n", i, g[i]);
}
