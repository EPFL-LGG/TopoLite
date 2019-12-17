//
// Created by ziqwang on 07.03.19.
//


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <string>
#include "IO/XMLIO.h"
#include "PyTopoCreator.h"
#include "PyContactGraph.h"
#include "PyInterlockChecker.h"


namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

PYBIND11_MODULE(pyTopo, m)
{
    m.doc() = "An interlocking Checker"; // optional module docstring

    py::class_<PyParamList>(m, "PyParamList")
            .def(py::init<>());

    py::class_<PyTopoCreator>(m, "PyTopoCreator")
            .def(py::init<const std::string &>())
            //.def("getStruc", &PyTopoCreator::getStruc)
            //.def("getParamList", &PyTopoCreator::getParamList)
            .def("numParts", &PyTopoCreator::numParts);

    py::class_<PyPolyMesh>(m, "PyPolyMesh")
            .def(py::init<const py::object &, bool, const PyParamList &>())
            .def(py::init<pPolyMesh , bool>())
            .def("convert2PolyMesh", &PyPolyMesh::convert2PolyMesh)
            .def("getCompasMesh", &PyPolyMesh::getCompasMesh);

    py::class_<PyContactGraph>(m, "PyContactGraph")
            .def(py::init<const vector<PyPolyMesh> &, float>())
            .def("getContacts", &PyContactGraph::getContacts)
            .def("numContacts", &PyContactGraph::numContacts);

    py::class_<PyInterlockCheck> interlockCheck(m, "PyInterlockCheck");

    interlockCheck.def(py::init<const PyContactGraph &, PyInterlockCheck::OptSolverType>())
                    .def("getInterlockingMat", &PyInterlockCheck::getInterlockingMat);

    py::enum_<PyInterlockCheck::OptSolverType>(interlockCheck, "OptSolverType")
            .value("CVXOPT", PyInterlockCheck::OptSolverType::CVXOPT)
            .value("MOSEK", PyInterlockCheck::OptSolverType::MOSEK)
            .export_values();
}