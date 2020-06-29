//
// Created by Robin Jodon on 19.04.20.
//

#ifndef TOPOLITE_IPOPT_TEST_PROBLEM_BIGM_H
#define TOPOLITE_IPOPT_TEST_PROBLEM_BIGM_H

#define HAVE_CSTDDEF
#include <IpTNLP.hpp>
#undef HAVE_CSTDDEF

using namespace Ipopt;

/**
 * @brief Simple test case of IPOPT solver
 *        We are solving the following LP problem with
 *        the big M matrix method
 *
 *        max 3x + 4y + 2z + \sum_{i=0}^{n var} L_i * M
 *    s.t.    2x           + L_1 = 4     E1
 *             x      + 2z + L_2 = 8     E2
 *                 3y +  z + L_3 = 6     E3
 *             (x,y,z, L_i)     >= 0     E4
 *
 *        Solution is: x,y,z = (2,1,3)
 *                     max   = 16
 *
 * @note This class is overloaded from TNLP interface
 */
class problem_LP_bigm : public TNLP {

public:
    /** solution vector - Used for the init point as well */
    std::vector<double> x_sol;
    Index hessian_triplets_nb;
    double big_m;

    /** Default constructor */
    problem_LP_bigm();

    /** Default destructor */
    ~problem_LP_bigm() override;


    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the NLP */
    bool get_nlp_info(
            Index &n,
            Index &m,
            Index &nnz_jac_g,
            Index &nnz_h_lag,
            IndexStyleEnum &index_style
    ) override;

    /** Method to return the bounds for my problem */
    bool get_bounds_info(
            Index n,
            Number *x_l,
            Number *x_u,
            Index m,
            Number *g_l,
            Number *g_u
    ) override;

    /** Method to return the starting point for the algorithm */
    bool get_starting_point(
            Index n,
            bool init_x,
            Number *x,
            bool init_z,
            Number *z_L,
            Number *z_U,
            Index m,
            bool init_lambda,
            Number *lambda
    ) override;

    /** Method to return the objective value */
    bool eval_f(
            Index n,
            const Number *x,
            bool new_x,
            Number &obj_value
    ) override;

    /** Method to return the gradient of the objective */
    bool eval_grad_f(
            Index n,
            const Number *x,
            bool new_x,
            Number *grad_f
    ) override;

    /** Method to return the constraint residuals */
    bool eval_g(
            Index n,
            const Number *x,
            bool new_x,
            Index m,
            Number *g
    ) override;

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    bool eval_jac_g(
            Index n,
            const Number *x,
            bool new_x,
            Index m,
            Index nele_jac,
            Index *iRow,
            Index *jCol,
            Number *values
    ) override;

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    bool eval_h(
            Index n,
            const Number *x,
            bool new_x,
            Number obj_factor,
            Index m,
            const Number *lambda,
            bool new_lambda,
            Index nele_hess,
            Index *iRow,
            Index *jCol,
            Number *values
    ) override;

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    void finalize_solution(
            SolverReturn status,
            Index n,
            const Number *x,
            const Number *z_L,
            const Number *z_U,
            Index m,
            const Number *g,
            const Number *lambda,
            Number obj_value,
            const IpoptData *ip_data,
            IpoptCalculatedQuantities *ip_cq
    ) override;
    //@}

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
    problem_LP_bigm(
            const problem_LP_bigm&
    );

    problem_LP_bigm& operator=(
            const problem_LP_bigm&
    );
    //@}
};


/**
 * @brief Simple test case of IPOPT solver
 *        We are solving the following LP problem with
 *        the big M matrix method
 *
 *        max 3x + 4y + 2z + \lambda * M
 *    s.t.    2x           + \lambda = 4     E1
 *             x      + 2z + \lambda = 8     E2
 *                 3y +  z + \lambda = 6     E3
 *             (x,y,z, L_i)     >= 0     E4
 *
 *        Solution is: x,y,z = (2,1,3)
 *                     max   = 16
 *
 * @note This class is overloaded from TNLP interface
 */
class problem_LP_bigm_unidim : public TNLP {

public:
    /** solution vector - Used for the init point as well */
    std::vector<double> x_sol;
    Index hessian_triplets_nb;
    double big_m;

    /** Default constructor */
    problem_LP_bigm_unidim();

    /** Default destructor */
    ~problem_LP_bigm_unidim() override;


    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the NLP */
    bool get_nlp_info(
            Index &n,
            Index &m,
            Index &nnz_jac_g,
            Index &nnz_h_lag,
            IndexStyleEnum &index_style
    ) override;

    /** Method to return the bounds for my problem */
    bool get_bounds_info(
            Index n,
            Number *x_l,
            Number *x_u,
            Index m,
            Number *g_l,
            Number *g_u
    ) override;

    /** Method to return the starting point for the algorithm */
    bool get_starting_point(
            Index n,
            bool init_x,
            Number *x,
            bool init_z,
            Number *z_L,
            Number *z_U,
            Index m,
            bool init_lambda,
            Number *lambda
    ) override;

    /** Method to return the objective value */
    bool eval_f(
            Index n,
            const Number *x,
            bool new_x,
            Number &obj_value
    ) override;

    /** Method to return the gradient of the objective */
    bool eval_grad_f(
            Index n,
            const Number *x,
            bool new_x,
            Number *grad_f
    ) override;

    /** Method to return the constraint residuals */
    bool eval_g(
            Index n,
            const Number *x,
            bool new_x,
            Index m,
            Number *g
    ) override;

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    bool eval_jac_g(
            Index n,
            const Number *x,
            bool new_x,
            Index m,
            Index nele_jac,
            Index *iRow,
            Index *jCol,
            Number *values
    ) override;

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    bool eval_h(
            Index n,
            const Number *x,
            bool new_x,
            Number obj_factor,
            Index m,
            const Number *lambda,
            bool new_lambda,
            Index nele_hess,
            Index *iRow,
            Index *jCol,
            Number *values
    ) override;

    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    void finalize_solution(
            SolverReturn status,
            Index n,
            const Number *x,
            const Number *z_L,
            const Number *z_U,
            Index m,
            const Number *g,
            const Number *lambda,
            Number obj_value,
            const IpoptData *ip_data,
            IpoptCalculatedQuantities *ip_cq
    ) override;
    //@}

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
    problem_LP_bigm_unidim(
            const problem_LP_bigm_unidim&
    );

    problem_LP_bigm_unidim& operator=(
            const problem_LP_bigm_unidim&
    );
    //@}
};

#endif

