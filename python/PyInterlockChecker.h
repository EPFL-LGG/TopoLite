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

    enum InterlockType{
        Translational = 0,
        Rotational = 1,
    };

    shared_ptr<InterlockingSolver<double>> solver;

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

    py::object checkInterlocking(InterlockType interlock_type){
        py::dict result;
        if(solver)
        {
            shared_ptr<InterlockingSolver<double>::InterlockingData> data;
            bool is_interlocking = false;
            if(interlock_type == Rotational)
                is_interlocking = solver->isRotationalInterlocking(data);
            else
                is_interlocking = solver->isTranslationalInterlocking(data);

            py::list py_trans, py_rot,py_center;

            if(is_interlocking)
            {
                result = py::dict("is_interlocking"_a = true);
            }
            else{

                for(int id = 0;id < data->traslation.size(); id++){
                    py::list vec;
                    vec.append(data->traslation[id].x());vec.append(data->traslation[id].y());vec.append(data->traslation[id].z());
                    py_trans.append(vec);
                }

                if(interlock_type == Rotational)
                {
                    for(int id = 0;id < data->rotation.size(); id++){
                        py::list vec;
                        vec.append(data->rotation[id].x());vec.append(data->rotation[id].y());vec.append(data->rotation[id].z());
                        py_rot.append(vec);
                    }

                    for(int id = 0;id < data->center.size(); id++){
                        py::list vec;
                        vec.append(data->center[id].x());vec.append(data->center[id].y());vec.append(data->center[id].z());
                        py_center.append(vec);
                    }
                    result = py::dict("is_interlocking"_a = false, "translation"_a = py_trans, "rotation"_a = py_rot, "rotation_center"_a = py_center);
                }
                else{
                    result = py::dict("is_interlocking"_a = false, "translation"_a = py_trans);
                }
            }
        }
        return result;
    }
};

#endif //TOPOLITE_PYINTERLOCKCHECKER_H
