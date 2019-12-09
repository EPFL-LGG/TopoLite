//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H
#define TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H

#include "ContactGraph.h"
#include "fusion.h"

class ContactGraphMosekSolver : public ContactGraph
{
public:

    float mosek_translational_interlocking_eps;

    float mosek_rotational_interlocking_eps;

    float mosek_rbe_eps;

    float mosek_intpntCoTolRelGap;

    float mosek_rbeForce_upperBound;

public:

    float slope_binarySearch_eps;

    float slope_bound_eps;

    float small_zero_eps;

    int   slope_num_uniform_sample;

    int   slope_num_init_sample;
public:

    ContactGraphMosekSolver();

public:

    void computeTranslationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i& size);

    void computeRotationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i& size);

public:

    bool isTranslationalInterlocking(string &log);

    bool isRotationalInterlocking(string &log);

    bool isEquilibrium(EigenPoint gravity, string &log, bool saveForce);

public:

    double computeSlope(EigenPoint gravity, EigenPoint laterial, double init_angle);

    double computeSlope(EigenPoint gravity);

    double computeSlopeUniform(EigenPoint gravity, int n_sample);

    std::tuple<EigenPoint, EigenPoint> computeSlopeCoordinateSystem(EigenPoint gravity);
};



#endif //TOPOLOCKCREATOR_CONTACTGRAPHMOSEKSOLVER_H
