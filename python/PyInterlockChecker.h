//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYINTERLOCKCHECKER_H
#define TOPOLITE_PYINTERLOCKCHECKER_H

#include "PyContactGraph.h"
#include "Interlocking/InterlockingSolver.h"
#include "Interlocking/InterlockingSolver_Clp.h"
#include "Interlocking/InterlockingSolver_AffineScaling.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyInterlockCheck {
public:
    enum OptSolverType{
        CLP = 0,
        AFFINE_SCALING = 1,
    };

    shared_ptr<InterlockingSolver<double>> solver;
    OptSolverType type;


public:

    PyInterlockCheck(const PyContactGraph &pygraph, OptSolverType solver_type)
    {
        switch (solver_type){
            case CLP:
                solver = make_shared<InterlockingSolver_Clp<double>>(pygraph.graph, pygraph.graph->getVarList());
                break;
            case AFFINE_SCALING:
                solver = make_shared<InterlockingSolver_AffineScaling<double>>(pygraph.graph, pygraph.graph->getVarList());
                break;
            default:
                solver = make_shared<InterlockingSolver<double>>(pygraph.graph, pygraph.graph->getVarList());
                break;
        }
        type = solver_type;
    }

public:

    Eigen::MatrixXd getInterlockingMat(bool Rotation = true)
    {
        Eigen::MatrixXd mat;
        if(solver)
        {
            if(Rotation)
                solver->computeRotationalInterlockingMatrixDense(mat);
            else
                solver->computeTranslationalInterlockingMatrixDense(mat);
        }
        return mat;
    }

    bool checkInterlocking(bool Rotation = true){
        if(solver)
        {
            shared_ptr<InterlockingSolver<double>::InterlockingData> data;
            if(Rotation)
                return solver->isRotationalInterlocking(data);
            else
                return solver->isTranslationalInterlocking(data);
        }
        return false;
    }
};

#endif //TOPOLITE_PYINTERLOCKCHECKER_H
