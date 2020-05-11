//
// Created by robinjodon on 29.04.20.
//
#include <cassert>
#include <iostream>
#include "tbb/tbb.h"

#include <IpIpoptApplication.hpp>
#include "InterlockingSolver_Ipopt.h"
#include <Eigen/SparseQR>
#include "Utility/SparseOperations.h"

using namespace Ipopt;


const double COIN_DBL_MAX = std::numeric_limits<double>::max();


/* ----------------------------------------------------------------------------------------------------------------- */
/* ----IPOPT INTERLOCKING SOLVER------------------------------------------------------------------------------------ */
/* ----------------------------------------------------------------------------------------------------------------- */

template<typename Scalar>
bool InterlockingSolver_Ipopt<Scalar>::isTranslationalInterlocking(InterlockingSolver_Ipopt::pInterlockingData &data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrix(tris, size);

    if (!checkSpecialCase(data, tris, false, size)) {
        return false;
    }

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, false);

    return solve(data, tris, false, size[0], size[1], num_var);
}

template<typename Scalar>
bool InterlockingSolver_Ipopt<Scalar>::isRotationalInterlocking(InterlockingSolver_Ipopt::pInterlockingData &data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(tris, size);

//    std::cout << "special case" << std::endl;
    if (!checkSpecialCase(data, tris, true, size)) {
        return false;
    }

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, true);

//    std::cout << "solve" << std::endl;
    return solve(data, tris, true, size[0], size[1], num_var);
}

template<typename Scalar>
bool InterlockingSolver_Ipopt<Scalar>::checkSpecialCase(pInterlockingData &data,
                                                        vector<EigenTriple> copy_tris,
                                                        bool rotationalInterlockingCheck,
                                                        Eigen::Vector2i copy_size) {
    return true;
}

/**
 * @brief UPDATE THIS 
 *
 *  Problem definition
 *  ------------------
 *
 *  - tris is equal to a sparse matrix A, which size is [num_row x num_col] 
 *  - our variables are [x, t], a row vector.
 *  - x: (size: num_var) is the instant translational and rotational velocity.
 *  - t: (size: num_col - num_var) is the auxiliary variable.
 *
 *  The optimization is formulated as:
 *
 *              min (-\sum_{i = 0}^{num_row} t_i)
 *  s.t.             A[x, t] >= 0
 *                  1 >= t >= 0
 *                    x \in R
 *
 *  Expected results
 *  ----------------
 *
 *  - Ideally if the structure is interlocking, the objective value should be zero.
 *  - In practice, due to numerical error, we have to allow a small tolerance for the objective value.
 */
template<typename Scalar>
bool InterlockingSolver_Ipopt<Scalar>::solve(InterlockingSolver_Ipopt::pInterlockingData &data, vector<EigenTriple> &tris,
                                             bool rotationalInterlockingCheck,
                                             int num_row,
                                             int num_col,
                                             int num_var) {


    // [0] - Instance for Ipopt App and NLP
    SmartPtr<IpoptProblem> interlock_pb = new IpoptProblem();       // problem to solve
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();     // solver

    // [1] - Define the matrix B
    EigenSpMat b(num_row, num_col);
    b.setFromTriplets(tris.begin(), tris.end());

    interlock_pb->initialize(b);

    // [2] - Set some options for the solver 
    app->Options()->SetNumericValue("tol", 1E-6);
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("linear_solver", "mumps");  // only available yet with installed IPOPT lib
    // app->Options()->SetStringValue("derivative_test", "first-order"); // excellent for debugging

    // [3] - Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        printf("\n\n*** Error during initialization!\n");
    }
    
    // [5] - Optimzation
    status = app->OptimizeTNLP(interlock_pb);

    if (status == Solve_Succeeded) {
        printf("\n\n*** The problem solved!\n");
    } else {
        printf("\n\n*** The problem FAILED!\n");
    }

    // unpackSolution(data, rotationalInterlockingCheck, solution, num_var);
    if(interlock_pb->max_abs_t < 1E-4){
        return true;
    }
    else{
        return false;
    }
}

template<typename Scalar>
void InterlockingSolver_Ipopt<Scalar>::unpackSolution(InterlockingSolver_Ipopt::pInterlockingData &data,
                                                      bool rotationalInterlockingCheck,
                                                      const double *solution,
                                                      int num_var) {
    data = make_shared<typename InterlockingSolver<Scalar>::InterlockingData>();
    for (pContactGraphNode node: graph->nodes) {
        Vector3 trans(0, 0, 0);
        Vector3 rotate(0, 0, 0);
        Vector3 center = node->centroid;
        if (node->dynamicID != -1) {
            if (rotationalInterlockingCheck) {
                trans = Vector3(solution[node->dynamicID * 6],
                                solution[node->dynamicID * 6 + 1],
                                solution[node->dynamicID * 6 + 2]);
                rotate = -Vector3(solution[node->dynamicID * 6 + 3],
                                  solution[node->dynamicID * 6 + 4],
                                  solution[node->dynamicID * 6 + 5]);
            } else {
                trans = Vector3(solution[node->dynamicID * 3],
                                solution[node->dynamicID * 3 + 1],
                                solution[node->dynamicID * 3 + 2]);
            }
        }

        data->traslation.push_back(trans);
        data->rotation.push_back(rotate);
        data->center.push_back(center);

//        std::cout << node->staticID << ":" << trans.transpose() << ", " << rotate.transpose() << std::endl;
    }
}


void TemporaryFunction_InterlockingSolver_Ipopt ()
{
    InterlockingSolver_Ipopt<double> solver(nullptr, nullptr);
}

/* ------------------------------------------------------------------------------------------------------------------ */
/* ----IPOPT INTERLOCKING PROBLEM------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------ */

/**
 * @brief Construct a new Ipopt Problem:: Ipopt Problem object
 * 
 */
IpoptProblem::IpoptProblem() {
    this->index_style = TNLP::C_STYLE;
    this->big_m = 5E6;
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

bool IpoptProblem::initialize(EigenSpMat &mat) {
    this->n_var_real = mat.cols() - mat.rows();         // variables without big M multiplier lambda and aux vars
    this->n_var = 1 + mat.cols();                       // variables including big M multiplier and auxiliary vars
    this->n_constraints = mat.rows();                   // Constraints inequalities

    mat.prune(0.0, 1E-9);
    this->b_coeff = mat; 

    set_vectors_dimensions();
    
    set_bounds_info();

    append_bigm_variables(mat);

    this->non_zero_jacobian_elements = this->b_coeff.nonZeros();     // non zero elements in Jacobian (initial B matrix + id(m,m))
    this->non_zero_hessian_elements = 0;                             // non-zero elements in Lagrangian Hessian

    return true;
}

void IpoptProblem::set_vectors_dimensions() {
    // allocate size for x vector
    this->x.resize(n_var);
    this->x_solution.resize(n_var);

    this->x_l.resize(this->n_var);
    this->x_u.resize(this->n_var);

    this->g_l.resize(this->n_constraints);
    this->g_u.resize(this->n_constraints);
}

void IpoptProblem::append_bigm_variables(EigenSpMat &mat) {
    this->b_coeff.conservativeResize(mat.rows(), this->n_var);
    this->b_coeff.reserve(mat.nonZeros() + mat.rows());
    for (int c = 0; c < mat.rows(); ++c) {
        this->b_coeff.insert(c, this->n_var-1) = 1.0;
    }
    this->b_coeff.finalize();
    this->b_coeff.prune(0.0, 1E-9);
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
    // Dynamic allocation

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
            // lambda >= 0
            this->x_l[i] = 0.0;
            this->x_u[i] = 1.1; //don't need to be inifinite, 1 is enough.
        }
    }
    return true;
}

// returns the initial point for the problem
bool IpoptProblem::get_starting_point(int n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                      int m, bool init_lambda, Number *lambda) {

    for (int i = 0; i < this->n_var; i++) {
        if (i < this->n_var_real)
        {
            // x_i is defined in R
            this->x[i] = x[i] = 0.0;
        }
        else
        {
            this->x[i] = x[i] = 1.0;
        }
    }
    return true;
}

// return the objective function
bool IpoptProblem::eval_f(int n, const Number *x, bool new_x, Number &obj_value) {
    this->obj_value = 0.0;
    for (int i = 0; i < this->n_var-1; i++) {
        if (i >= this->n_var_real)
            this->obj_value -= x[i];
    }
    this->obj_value += x[this->n_var-1] * this->big_m;
    
    obj_value = this->obj_value;
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool IpoptProblem::eval_grad_f(int n, const Number *x, bool new_x, Number *grad_f) {
    // minus everywhere --> searching for maximum
    // loop through n_var -1
    for (int i = 0; i < this->n_var-1; i++)
        if (i < this->n_var_real) {
            grad_f[i] = 0.0;
        } else {
            grad_f[i] = -1.0;
        }
    // Last constribition: the big_m
    grad_f[this->n_var-1] = this->big_m;
    return true;
}

// return the value of the constraints: g(x)
// Computes g = B * X
bool IpoptProblem::eval_g(int n, const Number *x, bool new_x, int m, Number *g) {
    Eigen::Map<const Eigen::VectorXd> vecx(x, n);
    VectorXd vecg = b_coeff * vecx;
    std::copy(vecg.data(), vecg.data() + m, g);
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
        // loop through B sparse elements and get values.
        int idx = 0;
        for (int k = 0; k < this->b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(this->b_coeff, k); it; ++it) {
                values[idx] = it.value();
                idx++;
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
        int idx = 0;
        for (int k = 0; k < this->b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(this->b_coeff, k); it; ++it) {
                values[idx] = 0;
            }
        }
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
    max_abs_t = 0;
//    printf("Solution of the primal variables, x\n");
    for (int i = 0; i < n; i++) {
        this->x[i] = x[i];
        this->x_solution[i] = x[i];
        if(i >= n_var_real && i < n_var - 1){
            max_abs_t = std::max(x[i], max_abs_t);
        }
    }

    printf("Maximum |t|:\t %E\n", max_abs_t);
    
    Number sum_g = 0;
    for (int i = 0; i < m; i++)
        sum_g+= std::abs(g[i]);
    printf("Sum of the final values of the constraints:\t %.3f\n", sum_g);
}