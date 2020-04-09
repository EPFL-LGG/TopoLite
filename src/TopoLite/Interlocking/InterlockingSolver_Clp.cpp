//
// Created by ziqwang on 04.04.20.
//


#include "InterlockingSolver_Clp.h"
#include "tbb/tbb.h"

template<typename Scalar>
bool InterlockingSolver_Clp<Scalar>::isTranslationalInterlocking(InterlockingSolver_Clp::pInterlockingData& data)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrix(tris, size);

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, false);

    return solve(data, tris, false, size[0], size[1], num_var);
}

template<typename Scalar>
bool InterlockingSolver_Clp<Scalar>::isRotationalInterlocking(InterlockingSolver_Clp::pInterlockingData& data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(tris, size);

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, true);

    return solve(data, tris, false, size[0], size[1], num_var);
}

template<typename Scalar>
bool InterlockingSolver_Clp<Scalar>::solve(     InterlockingSolver_Clp::pInterlockingData& data,
                                                vector<EigenTriple> &tris,
                                                bool rotationalInterlockingCheck,
                                                int num_row,
                                                int num_col,
                                                int num_var)
{

    //Problem definition
    //tris is equal to a sparse matrix A, which size is [num_row x num_col]
    //our variables are [x, t], a row vector.
    //x: (size: num_var) is the instant translational and rotational velocity.
    //t: (size: num_col - num_var) is the auxiliary variable.
    //the optimization is formulated as:
    //              min \sum_{i = 0}^{num_row} -t_i
    //  s.t.            A[x, t] >= 0
    //                  1 >= t >= 0
    //                    x \in R
    // Ideally if the structure is interlocking, the objective value should be zero.
    // In practice, due to numerical error, we have to allow a small tolerance for the objective value.


    EigenSpMat spatMat(num_row, num_col);
    spatMat.setFromTriplets(tris.begin(), tris.end());

    ClpSimplex model(true);
    CoinPackedMatrix matrix(true, num_row, num_col, spatMat.nonZeros(), spatMat.valuePtr(), spatMat.innerIndexPtr(), spatMat.outerIndexPtr(), spatMat.innerNonZeroPtr());

    double* objective = new double[num_col];
    double* rowLower = new double[num_row];
    double* rowUpper = new double[num_row];
    double* colLower = new double[num_col];
    double* colUpper = new double[num_col];

    //objects
    for(size_t id = 0; id < num_col; id++)
    {
        if(id < num_var) objective[id] = 0;
        else objective[id] = -1;
    }

    //bound
    for(size_t id = 0; id < num_row; id++) rowLower[id] = 0;
    for(size_t id = 0; id < num_row; id++) rowUpper[id] = 0;
    for(size_t id = 0; id < num_col; id++) {
        if(id < num_var) colLower[id] = -COIN_DBL_MAX;
        else colLower[id] = 0;
    }
    for(size_t id = 0; id < num_col; id++) {
        if(id < num_var) colUpper[id] = COIN_DBL_MAX;
        else colUpper[id] = 1;
    }

    CoinMessageHandler handler;
    handler.setLogLevel(0); //set loglevel to zero will silence the solver
    model.passInMessageHandler(& handler);
    model.newLanguage(CoinMessages::us_en);

    // load problem
    model.loadProblem(matrix, colLower, colUpper, objective, rowLower, rowUpper);

    // set tolerance
    // a experiment discovery: if the structure is interlocking,
    // the maximum "t" (the auxiliary variables) is around tolerance * 10
    // the average of the "t" is around tolerance * 5.
    // it is very useful to use these number to check whether structure is interlocking or not.
    model.setPrimalTolerance(1e-8);

    // Solve
    model.primal();

    // Solution
    const double target_obj_value = model.rawObjectiveValue();
    double *solution = model.primalColumnSolution();
    double max_sol = 0;
    for(int id = num_var; id < num_col; id++){
        max_sol = std::max(solution[id], max_sol);
    }

    data = make_shared<typename InterlockingSolver<Scalar>::InterlockingData>();

    for(pContactGraphNode node: graph->nodes){
        Vector3 trans(0, 0, 0);
        Vector3 rotate(0, 0, 0);
        Vector3 center(0, 0, 0);
        if(node->dynamicID != -1) {
            if (rotationalInterlockingCheck) {
                trans = Vector3(solution[node->dynamicID * 6],
                        solution[node->dynamicID * 6 + 1],
                        solution[node->dynamicID * 6 + 2]);
                rotate = Vector3(solution[node->dynamicID * 6 + 3],
                        solution[node->dynamicID * 6 + 4],
                        solution[node->dynamicID * 6 + 5]);
            }
            else{
                trans = Vector3(solution[node->dynamicID * 3],
                        solution[node->dynamicID * 3 + 1],
                        solution[node->dynamicID * 3 + 2]);
            }
            center = node->centroid;
        }
        data->traslation.push_back(trans);
        data->rotation.push_back(rotate);
        data->center.push_back(center);
//        std::cout << node->dynamicID << ":\t" << trans.transpose() << ", " << rotate.transpose() << std::endl;
    }

    std::cout << "max_t:\t" << std::abs(max_sol) << std::endl;
    std::cout << "average_t:\t" << std::abs(target_obj_value) / num_row << std::endl;
    if(max_sol < 5e-6) {
        return true;
    }
    else{
        return false;
    }
}
