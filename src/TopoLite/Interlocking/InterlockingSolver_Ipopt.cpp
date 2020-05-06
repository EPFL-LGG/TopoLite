//
// Created by robinjodon on 29.04.20.
//
#include <IpIpoptApplication.hpp>
#include "InterlockingSolver_Ipopt.h"
#include "Ipopt_Interlocking_Problem.h"

#include "tbb/tbb.h"
#include <Eigen/SparseQR>


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

    // [0] - Define the matrix B
    EigenSpMat b(num_row, num_col);
    b.setFromTriplets(tris.begin(), tris.end());

    // [1] - Instance for Ipopt App and NLP
    SmartPtr<IpoptProblem> interlock_pb = new IpoptProblem();       // problem to solve
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();     // solver

    interlock_pb->initialize(num_var, num_col, b);

    // [2] - Set some options for the solver 
    app->Options()->SetNumericValue("tol", 1e-7);
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
    return true;
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
