//
// Created by ziqwang on 04.04.20.
//


template<typename Scalar>
bool InterlockingSolver_Clp<Scalar>::isTranslationalInterlocking(InterlockingSolver_Clp::pInterlockingData data)
{
    return true;
}

template<typename Scalar>
bool InterlockingSolver_Clp<Scalar>::isRotationalInterlocking(InterlockingSolver_Clp::pInterlockingData data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(tris, size);
    std::cout << size.transpose() << std::endl;

    int num_row = size[0];
    int num_col = size[1];
    int num_col_with_auxiliary = num_row + num_col;

    for(size_t id = 0; id < num_row; id++){
        tris.push_back(EigenTriple(id, num_col + id, -1));
    }

    EigenSpMat spatMat(num_row, num_col_with_auxiliary);
    spatMat.setFromTriplets(tris.begin(), tris.end());

    ClpSimplex model;
    CoinPackedMatrix matrix(true, num_row, num_col_with_auxiliary, spatMat.nonZeros(), spatMat.valuePtr(), spatMat.innerIndexPtr(), spatMat.outerIndexPtr(), spatMat.innerNonZeroPtr());

    double* objective = new double[num_col_with_auxiliary];
    double* rowLower = new double[num_row];
    double* rowUpper = new double[num_row];
    double* colLower = new double[num_col_with_auxiliary];
    double* colUpper = new double[num_col_with_auxiliary];
    //objects
    for(size_t id = 0; id < num_col_with_auxiliary; id++)
    {
        if(id < num_col) objective[id] = 0;
        else objective[id] = -1;
    }

    //bound
    for(size_t id = 0; id < num_row; id++) rowLower[id] = 0;
    for(size_t id = 0; id < num_row; id++) rowUpper[id] = COIN_DBL_MAX;
    for(size_t id = 0; id < num_col_with_auxiliary; id++) {
        if(id < num_col) colLower[id] = -COIN_DBL_MAX;
        else colLower[id] = 0;
    }
    for(size_t id = 0; id < num_col_with_auxiliary; id++) {
        if(id < num_col) colUpper[id] = COIN_DBL_MAX;
        else colUpper[id] = 10;
    }

    // load problem
    model.loadProblem(matrix, colLower, colUpper, objective, rowLower, rowUpper);
    // Solve
    model.setPrimalTolerance(1e-7);
    model.primal();
    // Solution
    const double target_obj_value = model.rawObjectiveValue();
    std::cout << target_obj_value << std::endl;
    double *solution = model.primalColumnSolution();
    for(int id = 0; id < num_col_with_auxiliary; id++){
        if(id < num_col) {
            std::cout << solution[id] << std::endl;
        }
    }
    if(target_obj_value < 1e-7) {
        return true;
    }
    else{
        return false;
    }

}