//
// Created by ziqwang on 2019-12-15.
//

#ifndef TOPOLITE_PYPOLYMESH_H
#define TOPOLITE_PYPOLYMESH_H
#include "Mesh/PolyMesh.h"
#include "Mesh/MeshConverter.h"
#include "Utility/TopoObject.h"

#include "PyParamList.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyPolyMesh{
public:

    shared_ptr<PolyMesh> mesh_;

    bool atBoundary_;

    PyPolyMesh(){


    }

    PyPolyMesh(const py::object &compas_mesh, bool atBoundary, const PyParamList &varList)
    {
        setCompasMesh(compas_mesh, varList);
        atBoundary_ = atBoundary;
    }

    PyPolyMesh(pPolyMesh polymesh, bool atBoundary){
        mesh_ = make_shared<PolyMesh>(*polymesh);
        atBoundary_ = false;
    }

public:

    void convert2PolyMesh(double eps)
    {
        if(mesh_ == nullptr) return;
        MeshConverter converter(mesh_->getVarList());
        converter.Convert2PolyMesh(mesh_, eps);
    }

    py::object getCompasMesh()
    {
        if(mesh_ == nullptr) return py::object();

        py::list vs;
        py::list fs;

        for(size_t id = 0; id < mesh_->vertexList.size(); id++){
            py::list v;
            v.append(mesh_->vertexList[id].x);
            v.append(mesh_->vertexList[id].y);
            v.append(mesh_->vertexList[id].z);
            vs.append(v);
        }
        for(size_t id = 0; id < mesh_->polyList.size(); id++){
            py::list f;
            for(size_t jd = 0; jd < mesh_->polyList[id]->verIDs.size(); jd++){
                f.append(mesh_->polyList[id]->verIDs[jd]);
            }
            fs.append(f);
        }

        py::object module_compas_datastructures;
        py::object module_compas_mesh;
        module_compas_datastructures = py::module::import("compas.datastructures");
        module_compas_mesh = module_compas_datastructures.attr("Mesh");
        py::object pyMesh = module_compas_mesh.attr("from_vertices_and_faces")(vs, fs);

        return pyMesh;
    }

private:

    void setCompasMesh(const py::object &compasMesh, const PyParamList &varList)
    {
        py::list vfs = compasMesh.attr("to_vertices_and_faces")();
        py::list vs = vfs[0];
        py::list fs = vfs[1];

        mesh_ = make_shared<PolyMesh>(varList.data_);
        mesh_->vertexList.resize(py::len(vs));

        for(int id = 0; id < py::len(vs); id++){
            py::list v = vs[id];
            //py::print(v);
            Vector3f pt;
            for(int kd = 0; kd < 3; kd++){
                pt[kd] = py::float_(v[kd]);
            }
            mesh_->vertexList[id] = pt;
        }

        for(int id = 0; id < py::len(fs); id++)
        {
            py::list f = fs[id];
            //py::print(f);
            shared_ptr<_Polygon> poly = make_shared<_Polygon>();
            for(int kd = 0; kd < len(f); kd++){
                int vid = py::int_(f[kd]);
                poly->verIDs.push_back(vid);
                poly->vers.push_back(mesh_->vertexList[vid]);
            }
            mesh_->polyList.push_back(poly);
        }

        mesh_->removeDuplicatedVertices();
    }

};


#endif //TOPOLITE_PYPOLYMESH_H
