//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYINTERLOCKCHECKER_H
#define TOPOLITE_PYINTERLOCKCHECKER_H

#include "PyContactGraph.h"
#include "Interlocking/InterlockingSolver.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyInterlockCheck: public InterlockingSolver{

public:
    enum OptSolverType{
        CVXOPT = 0,
        MOSEK = 1,
    };

    OptSolverType type;

public:

    PyInterlockCheck(const PyContactGraph &pygraph, OptSolverType solver_type)
    : InterlockingSolver(pygraph.graph, pygraph.graph->getVarList())
    {
        type = solver_type;
    }

public:

    Eigen::MatrixXd getInterlockingMat(bool Rotation = true)
    {
        Eigen::MatrixXd mat;
        if(graph)
        {
            if(Rotation)
                computeRotationalInterlockingMatrixDense(mat);
            else
                computeTranslationalInterlockingMatrixDense(mat);
        }
        return mat;
    }
};

#endif //TOPOLITE_PYINTERLOCKCHECKER_H
