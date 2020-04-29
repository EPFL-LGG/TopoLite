//
// Created by Robin Jodon on 29.04.20.
//

#ifndef TOPOLITE_IPOPT_INTERLOCK_PROBLEM_H
#define TOPOLITE_IPOPT_INTERLOCK_PROBLEM_H

#define HAVE_CSTDDEF

#include <IpTNLP.hpp>

#undef HAVE_CSTDDEF

using namespace Ipopt;

/**
 * @brief  Interace between Ipopt NLP interface and TopoLite Interlocking Solver
 *         - NLP provides an easy interface to Ipopt.
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
class IpoptInterlockProblem : public TNLP {

public:
    /** solution vector - Used for the init point as well */
    std::vector<double> x_sol;

    /** Default constructor */
    IpoptInterlockProblem();

    /** Default destructor */
    ~IpoptInterlockProblem() override;


    /**
     * @brief Get infos about problem dimensions
     *
     * @param n number of variables
     * @param m number of constraints eq/ineq
     * @param nnz_jac_g non-zero elements in constaints Jacobian
     * @param nnz_h_lag non-zero elements in Lagrangian and Hessian
     * @param index_style C or Fortran style
     */
    bool get_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag, IndexStyleEnum &index_style) override;

    /**
     * @brief Set infos about problem dimensions. The following vars are set:
     *
     * @param n number of variables
     * @param m number of constraints eq/ineq
     * @param nnz_jac_g non-zero elements in constaints Jacobian
     * @param nnz_h_lag non-zero elements in Lagrangian and Hessian
     * @param index_style C or Fortran style
     */
    bool set_nlp_info(Index &n, Index &m, Index &nnz_jac_g, Index &nnz_h_lag, IndexStyleEnum &index_style);

    /**
     * @brief Get the problem bounds conditions
     *
     * @param n number of variables
     * @param x_l lower bounds for the variables
     * @param x_u upper bounds for the variables
     * @param m number of constraints eq/ineq
     * @param g_l lower bounds for the constraints
     * @param g_u upper bounds for the constraints
     */
    bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) override;


    /**
     * @brief Set the problem bounds conditions. The following vars are set.
     *
     * @param n number of variables
     * @param x_l lower bounds for the variables
     * @param x_u upper bounds for the variables
     * @param m number of constraints eq/ineq
     * @param g_l lower bounds for the constraints
     * @param g_u upper bounds for the constraints
     */
    bool set_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u);

    /**
     * @brief  Get the starting point for solving the pb
     *         Cannot be used for our problem. If we know a starting point, it is not interlocking.
     *
     * @param n number of variables
     * @param init_x
     * @param x
     * @param init_z
     * @param z_L
     * @param z_U
     * @param m
     * @param init_lambda
     * @param lambda
     */
    bool get_starting_point(Index n, bool init_x, Number *x, bool init_z, Number *z_L, Number *z_U, Index m, bool init_lambda, Number *lambda) override;

    /**
     * @brief
     *
     * @param n number of variables
     * @param x variables values
     * @param new_x
     * @param obj_value value of the objective function
     * @return
     */
    bool eval_f(Index n, const Number *x, bool new_x, Number &obj_value) override;

    /**
     * @brief Method to return the gradient of the objective
     * @param n number of variables
     * @param x variables values
     * @param new_x
     * @param grad_f gradient vector values
     * @return
     */
    bool eval_grad_f(Index n, const Number *x, bool new_x, Number *grad_f) override;

    /**
     * @brief Method to return the constraint residuals
     * @param n number of variables
     * @param x variables values
     * @param new_x
     * @param m number of constraints
     * @param g constraints residuals
     * @return
     */
    bool eval_g(Index n, const Number *x, bool new_x, Index m, Number *g) override;

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    bool eval_jac_g(Index n, const Number *x, bool new_x, Index m, Index nele_jac, Index *iRow, Index *jCol, Number *values) override;

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    bool eval_h(Index n, const Number *x, bool new_x, Number obj_factor, Index m, const Number *lambda, bool new_lambda, Index nele_hess,
                Index *iRow, Index *jCol, Number *values) override;

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    void finalize_solution(SolverReturn status, Index n, const Number *x, const Number *z_L, const Number *z_U, Index m, const Number *g,
                           const Number *lambda, Number obj_value, const IpoptData *ip_data,
                           IpoptCalculatedQuantities *ip_cq) override;     //@}

private:
    /**@name Methods to block default compiler methods.
     *
     * The compiler automatically generates the following three methods.
     *  Since the default compiler implementation is generally not what
     *  you want (for all but the most simple classes), we usually
     *  put the declarations of these methods in the private section
     *  and never implement them. This prevents the compiler from
     *  implementing an incorrect "default" behavior without us
     *  knowing. (See Scott Meyers book, "Effective C++")
     */
    //@{
    IpoptInterlockProblem(
            const IpoptInterlockProblem &
    );

    IpoptInterlockProblem &operator=(
            const IpoptInterlockProblem &
    );
    //@}
};

#endif

