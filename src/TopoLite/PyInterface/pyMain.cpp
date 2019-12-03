//
// Created by ziqwang on 07.03.19.
//


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <string>
#include <tbb/tbb.h>

#include "PyInterface.h"
#include "CrossMesh/PatternCreator.h"
#include "Structure/StrucCreator.h"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

/*
 * GLOBAL VARIABLES
 */
gluiVarList varList; //gluiVar
double interactMatrix[16] = {5,0,0,0, 0,5,0,0, 0,0,1,0, 0,0,0,1}; // pattern settings
shared_ptr<StrucCreator> myStrucCreator; // TI Assembly Creator
pugi::xml_document xmldoc;
vector<int> pickPartIDs;

PYBIND11_MODULE(pyTopoCreator, m)
{
    m.doc() = "TopoCreator Read File Test"; // optional module docstring

    m.def("read", [&](string filename) -> int {
        myStrucCreator.reset();
        myStrucCreator = make_shared<StrucCreator>();
        InitVar(varList);
        return ReadFile(filename);
    });

    m.def("save", [&](string filename) -> int {
        return SaveFile(filename);
    });

    m.def("clear", [&](){
        myStrucCreator.reset();
    });

    m.def("n_part", [&]() -> int{
        if(myStrucCreator && myStrucCreator->myStruc)
        {
            return myStrucCreator->myStruc->partList.size();
        }
        return -1;
    });

    m.def("n_boundary", [&]() -> int{
        if(myStrucCreator && myStrucCreator->myStruc)
        {
            return myStrucCreator->myStruc->partList.size() - myStrucCreator->myStruc->parts.size();
        }
        return -1;
    });

    m.def("interactive_time", [&](int N = 20) -> double{
        double interac_time = 0;
        if(myStrucCreator && myStrucCreator->myStruc){
            for(int id = 0; id < N ; id++){
                tbb::tick_count sta = tbb::tick_count::now();
                myStrucCreator->CreateStructure(true, true, interactMatrix, true);
                interac_time += (tbb::tick_count::now() - sta).seconds();
            }
            interac_time /= N;
        }
        return interac_time;
    });

    m.def("set_para_float", [&](string name, float value){
        varList.set(name, value);
    });

    m.def("set_para_int", [&](string name, int value){
        varList.set(name, value);
    });

    m.def("set_para_bool", [&](string name, bool value){
        varList.set(name, value);
    });

    m.def("get_para_float", [&](string name) ->float {
        return varList.get<float>(name);
    });

    m.def("get_para_int", [&](string name) ->int {
        return varList.get<int>(name);
    });

    m.def("contactArea", [&]() -> std::vector<double>
    {
        std::vector<double> areas;
        if(myStrucCreator && myStrucCreator->myStruc){
            for(int id = 0; id < myStrucCreator->myStruc->contactList.size(); id++)
            {
                shared_ptr<Contact> contact = myStrucCreator->myStruc->contactList[id];
                shared_ptr<Part> partA = myStrucCreator->myStruc->partList[contact->partIDA];
                shared_ptr<Part> partB = myStrucCreator->myStruc->partList[contact->partIDB];
                if(!partA->atBoundary || !partB->atBoundary)
                    areas.push_back(contact->area);
            }
        }
        return areas;
    });

    m.def("average_face_size", [&]() -> double{
        if(myStrucCreator && myStrucCreator->myStruc){
            return myStrucCreator->myStruc->average_face_area;
        }
        return 0.0;
    });

    m.def("get_faceface_contacts", [&]() -> int {
        if(myStrucCreator && myStrucCreator->myStruc){
            return myStrucCreator->myStruc->num_face_face;
        }
        return -1;
    });

    m.def("get_edgeedge_contacts", [&]() -> int {
        if(myStrucCreator && myStrucCreator->myStruc){
            return myStrucCreator->myStruc->num_edge_edge;
        }
        return -1;
    });

    py::class_<MeshCreator>(m, "Pattern2DCreator")
    .def(py::init<>())
    .def("create", &MeshCreator::PyCreateMesh_2DPattern);
}
