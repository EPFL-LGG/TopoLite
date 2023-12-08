//
// Created by robinjodon on 29.04.20.
//
#include <cassert>
#include <iostream>
#include "tbb/tbb.h"
#include "InterlockingSolver_Ipopt.h"
#include <Eigen/SparseQR>
#include "Utility/SparseOperations.h"

#define HAVE_CSTDDEF
#include <IpIpoptApplication.hpp>
#undef HAVE_CSTDDEF

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
    app->Options()->SetIntegerValue("print_level", 5);

    // C.2 Termination
    app->Options()->SetNumericValue("tol", 1e-5);
    app->Options()->SetNumericValue("acceptable_tol", 1e-5);

    // C.4 NLP
    app->Options()->SetStringValue("jac_c_constant", "yes");
    app->Options()->SetStringValue("jac_d_constant", "yes");
    app->Options()->SetStringValue("hessian_constant", "yes");

    // C.7 Multiplier update
    app->Options()->SetStringValue("alpha_for_y", "primal-and-full");   // step size use the primal step size and full step if delta x ¡= alpha for y tol
    app->Options()->SetNumericValue("alpha_for_y_tol", 100);            // Tolerance for switching to full equality multiplier steps

    // C.8 Line search 
    app->Options()->SetIntegerValue("max_soc", 0);                      // Disable 2ndOrder correction for trial steps at each iter.

    // C.10 Restoration phase
    app->Options()->SetStringValue("expect_infeasible_problem", "yes"); // enable heuristics to detect infeasibility quicker

    // todo: Exiting if t > 0 and lambda = 0 

    // C.11 Linear Solver 
    app->Options()->SetStringValue("linear_solver", "mumps");           // only available yet with installed IPOPT lib
    app->Options()->SetIntegerValue("min_refinement_steps", 0);         // iterative refinement steps/linear solve. Default=1 (changes Sum of the final values of constraints)
    app->Options()->SetIntegerValue("max_refinement_steps", 5);         // 
    // C.20 MUMPS settings 
    //  pivot_order is the most significant parameter
    //  * 1, 3 not accessible (can't install SCOTCH, 1 is manual mode)
    //  * 2 (AMF) is very slow (approximate minimum fill)
    //  * 4 (PORD) is slow.
    //  * 0 (AMD) quite fast (approximate minimum degree ordering)
    //  * 5 (METIS) is slow
    //  * 6 (QAMD the fastest (aproximate minimum degree ordering + quasi-dense row detection)
    //  * 77 (auto) seems to use AMD
    app->Options()->SetIntegerValue("mumps_pivot_order", 6);            // 0, 2, 6 are showing best perfs           (see MUMPS ICNTL(7))
    app->Options()->SetIntegerValue("mumps_scaling",   4);             // Huge differences (7 and 8 are slower),    (see MUMPS ICNTL(8))

    // C.6 Barrier param
    bool mehrotra = false;
    if (mehrotra) { 
        // Runs Mehrotra predictor-corrector algo. Works well with LPs. A bit more aggressive but slightly slower than adaptive coupled with LOQO. 
        app->Options()->SetStringValue("mehrotra_algorithm", "yes");
        app->Options()->SetNumericValue("tol", 1e+1);
        app->Options()->SetNumericValue("acceptable_tol", 1e+1);
        app->Options()->SetIntegerValue("min_refinement_steps", 0);         // iterative refinement steps/linear solve. Default=1 (changes Sum of the final values of constraints)
        app->Options()->SetIntegerValue("max_refinement_steps", 5);         // 
    } else { 
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        /* Oracle for the new barrier param - Deetermines how the new barrier is computed in each "free mode" iteration
         * probing/loqo/quality-function (default) */
        app->Options()->SetStringValue("mu_oracle", "loqo");
    }

    
    // For debugging purposes
    // app->Options()->SetStringValue("derivative_test", "first-order");


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

    unpackSolution(data, rotationalInterlockingCheck, interlock_pb->x_solution.data(), num_var);
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
        Vector3 center = (node->centroid).template cast<double>();
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

/* ------------------------------------------------------------------------------------------------------------------ */
/* ----IPOPT INTERLOCKING PROBLEM------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------ */


IpoptProblem::IpoptProblem() {
    index_style = TNLP::C_STYLE;
    big_m = 5E7;                    // a smaller bigM works 5e6, solving is faster but final lambda is not exactly 0 
}

// destructor
IpoptProblem::~IpoptProblem() = default;


// returns the problem dimensions
bool IpoptProblem::get_nlp_info(int &n, int &m, int &nnz_jac_g, int &nnz_h_lag, IndexStyleEnum &_index_style) {
    n = n_var;                                    // N+M variables [x, t, lamdba]
    m = n_constraints;                            // M inequalities
    nnz_jac_g = non_zero_jacobian_elements;       // non zero elements in Jacobian
    nnz_h_lag = non_zero_hessian_elements;        // non-zero elements in Lagrangian Hessian
    _index_style = index_style;                    // use the C style indexing (0-based)

    return true;
}

bool IpoptProblem::initialize(EigenSpMat &mat) {
    n_var_real = mat.cols() - mat.rows();         // variables without big M multiplier lambda and aux vars
    n_var = 1 + mat.cols();                       // variables including big M multiplier and auxiliary vars
    n_constraints = mat.rows();                   // Constraints inequalities

    mat.prune(0.0, 1E-9);
    b_coeff = mat; 

    set_vectors_dimensions();
    
    set_bounds_info();

    append_bigm_variables(mat);

    non_zero_jacobian_elements = b_coeff.nonZeros();     // non zero elements in Jacobian (initial B matrix + id(m,m))
    non_zero_hessian_elements = 0;                             // non-zero elements in Lagrangian Hessian

    return true;
}

void IpoptProblem::set_vectors_dimensions() {
    // allocate size for x vector
    x.resize(n_var);
    x_solution.resize(n_var);

    x_l.resize(n_var);
    x_u.resize(n_var);

    g_l.resize(n_constraints);
    g_u.resize(n_constraints);
}

void IpoptProblem::append_bigm_variables(EigenSpMat &mat) {
    b_coeff.conservativeResize(mat.rows(), n_var);
    b_coeff.reserve(mat.nonZeros() + mat.rows());
    for (int c = 0; c < mat.rows(); ++c) {
        b_coeff.insert(c, n_var-1) = 1.0;
    }
    b_coeff.finalize();
    b_coeff.prune(0.0, 1E-9);
}

// returns the variable bounds
bool IpoptProblem::get_bounds_info(int n, Number *x_l, Number *x_u, int m, Number *g_l, Number *g_u) {

    for (int i = 0; i < n_var; i++) {
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

    for (int i = 0; i < n_constraints; i++) {
        g_l[i] = 0;
        g_u[i] = COIN_DBL_MAX;
    }

    for (int i = 0; i < n_var; i++) {
        if (i < n_var_real) {
            // x_i is defined in R
            x_l[i] = COIN_DBL_MAX * (-1.0);
            x_u[i] = COIN_DBL_MAX;
        } else {
            // lambda and aux variables >= 0
            x_l[i] = 0.0;
            x_u[i] = COIN_DBL_MAX;  //don't need to be inifinite, 1 is enough. However unbounding reduce execution time
        }
    }
    return true;
}

// returns the initial point for the problem
bool IpoptProblem::get_starting_point(int n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U,
                                      int m, bool init_lambda, Number *lambda) {

    for (int i = 0; i < n_var; i++) {
        if (i < n_var_real) {
            this->x[i] = x[i] = 0.0;
        } else {
            this->x[i] = x[i] = 0.0;  // note: setting 0 instead of 1 makes the code faster
        }
    }
    return true;
}

// return the objective function
bool IpoptProblem::eval_f(int n, const Number *x, bool new_x, Number &obj_value) {
    this->obj_value = 0.0;
    for (int i = 0; i < n_var-1; i++) {
        if (i >= n_var_real)
            obj_value -= x[i];
    }
    this->obj_value += x[this->n_var-1] * this->big_m;
    
    obj_value = this->obj_value;
    return true;
}

// return the gradient of the objective function grad_{x} f(x)
bool IpoptProblem::eval_grad_f(int n, const Number *x, bool new_x, Number *grad_f) {
    // minus everywhere --> searching for maximum
    // loop through n_var -1
    for (int i = 0; i < n_var-1; i++)
        if (i < n_var_real) {
            grad_f[i] = 0.0;
        } else {
            grad_f[i] = -1.0;
        }
    // Last constribition: the big_m
    grad_f[n_var-1] = big_m;
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
        for (int k = 0; k < b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(b_coeff, k); it; ++it) {
                iRow[idx] = it.row();
                jCol[idx] = it.col();
                idx++;
            }
        }
    } else {
        // loop through B sparse elements and get values.
        int idx = 0;
        for (int k = 0; k < b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(b_coeff, k); it; ++it) {
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
        for (int row = 0; row < non_zero_hessian_elements; row++) {
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
        for (int k = 0; k < b_coeff.outerSize(); ++k) {
            for (SparseMatrix<double>::InnerIterator it(b_coeff, k); it; ++it) {
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
        this->x[i] = x_solution[i] = x[i];
        if(i >= n_var_real && i < n_var - 1){
            max_abs_t = std::max(x[i], max_abs_t);
        }
    }

    printf("Maximum |t|:\t %E\n", max_abs_t);
    printf("Lambda     :\t %E\n", x[n_var-1]);
    
    Number sum_g = 0;
    for (int i = 0; i < m; i++)
        sum_g+= std::abs(g[i]);
    printf("Sum of the final values of the constraints:\t %.3f\n", sum_g);
}


void TemporaryFunction_InterlockingSolver_Ipopt ()
{
    InterlockingSolver_Ipopt<double> solver(nullptr, nullptr);
}

template class InterlockingSolver_Ipopt<float>;
template class InterlockingSolver_Ipopt<double>;