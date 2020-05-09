//
// Created by ziqwang on 2020-02-03.
//
// bug: we keep this here for now but it is not working.
//
#include "InterlockingSolver_AffineScaling.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

template<typename Scalar>
bool InterlockingSolver_AffineScaling<Scalar>::isTranslationalInterlocking(pInterlockingData &data)
{
    return false;
}

template<typename Scalar>
bool InterlockingSolver_AffineScaling<Scalar>::isRotationalInterlocking(pInterlockingData &data)
{
    MatrixXd mat;
    InterlockingSolver<Scalar>::computeRotationalInterlockingMatrixDense(mat);

    int m = mat.rows();
    int n = mat.cols();

    VectorXd x0p = VectorXd::LinSpaced(n, 1, 1);
    VectorXd x0n = VectorXd::LinSpaced(n, 1, 1);
    VectorXd t0 = VectorXd::LinSpaced(m, 1, 1);
    double lambda0 = 1;

    VectorXd mu = t0 - mat*(x0p - x0n);
    MatrixXd A = MatrixXd(m, n * 2 + m + 1);

    A.middleCols(0, n) = mat;
    A.middleCols(n, n) = -mat;
    A.middleCols(2 * n, m) = -MatrixXd::Identity(m, m);
    A.col(2 * n + m) = mu;

    EigenSpMat spatA = EigenSpMat(m, n * 2 + m + 1);
    spatA = A.sparseView();

    VectorXd x0(2 * n + m + 1);

    x0 << x0p,
    x0n,
    t0,
    lambda0;

    double M = 100000;
    VectorXd c(2 * n + m + 1);
    c << VectorXd::LinSpaced(2*n, 0, 0),
    -VectorXd::LinSpaced(m, 1, 1),
    M;

    int k = 0;
    vector<VectorXd> xs;
    vector<EigenSpMat> Xs;

    xs.push_back(x0);

    float beta = 0.5;
    while(k < 20)
    {
        EigenSpMat X = EigenSpMat(2 * n + m + 1, 2* n + m + 1);
        for(int id = 0; id < 2 * n + m + 1; id++){
            X.insert(id, id) = xs[k](id);
        }
        Xs.push_back(X);

        // y^k = (AX2kA^T)^{-1}AX2kc
        EigenSpMat X2k = X * X;
        EigenSpMat AX2k = spatA * X2k;
        EigenSpMat AX2kAT = AX2k * spatA.transpose();
        VectorXd AX2kc = AX2k * c;
        Eigen::SparseLU<EigenSpMat, Eigen::COLAMDOrdering<int>> solver;
        solver.compute(AX2kAT);
        VectorXd y = solver.solve(AX2kc);

        //r = c - A^Ty
        VectorXd r = c - spatA.transpose() * y;
        if(r.maxCoeff() < 0)
            break;

        VectorXd Xkr = X * r;
        float nrmXkr = Xkr.norm();
        float maxXkr = Xkr.maxCoeff();
        float theta = nrmXkr / maxXkr;

        if(1 - theta < 1e-7){
            beta = 1;
        }

        xs.push_back(xs[k] - beta * theta *  X2k * r / nrmXkr);
        beta = 0.5;

        std::cout << "(" <<  k << ") " << c.dot(xs[k + 1]) << std::endl;
        //VectorXd Vec = xs.back().segment(0, n) - xs.back().segment(n, n);
        //std::cout << "Motion: " << Vec.norm() << ",\tError: " << (mat  * Vec).norm() << std::endl;

        if (c.dot(xs[k + 1]) < -1e3)
        {
            return false;
        }

        if (xs[k + 1].segment(2 * n, m).maxCoeff() < 1e-4){
            return true;
        }
        else{
            k = k + 1;
        }
    }
    return false;
}


void TemporaryFunction_InterlockingSolver_AffineScaling ()
{
    InterlockingSolver_AffineScaling<double> solver(nullptr, nullptr);
}