//
// Created by ziqwang on 2019-12-15.
//

#ifndef TOPOLITE_PYPOLYMESH_H
#define TOPOLITE_PYPOLYMESH_H

#include "Mesh/PolyMesh.h"
#include "Utility/TopoObject.h"

#include "PyParamList.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyPolyMesh{
public:
    typedef shared_ptr<PolyMesh<double>> pPolyMesh;
    typedef shared_ptr<_Polygon<double>> pPolygon;
    typedef shared_ptr<VPoint<double>> pVertex;
    typedef Matrix<double, 3 ,1> Vector3;

public:

    pPolyMesh mesh_;

    bool atBoundary_;

    PyPolyMesh(){


    }

    PyPolyMesh(const py::object &compas_mesh, bool atBoundary, const PyParamList &varList)
    {
        setCompasMesh(compas_mesh, varList);
        atBoundary_ = atBoundary;
    }

    PyPolyMesh(pPolyMesh polymesh, bool atBoundary){
        mesh_ = make_shared<PolyMesh<double>>(*polymesh);
        atBoundary_ = false;
    }

public:

    void mergeFaces(double eps)
    {
        if(mesh_ == nullptr) return;
        mesh_->mergeFaces();
    }

    py::object getCompasMesh()
    {
        if(mesh_ == nullptr) return py::object();

        py::list vs;
        py::list fs;

        for(size_t id = 0; id < mesh_->vertexList.size(); id++){
            py::list v;
            v.append(mesh_->vertexList[id]->pos.x());
            v.append(mesh_->vertexList[id]->pos.y());
            v.append(mesh_->vertexList[id]->pos.z());
            vs.append(v);
        }

        for(size_t id = 0; id < mesh_->polyList.size(); id++){
            py::list f;
            for(size_t jd = 0; jd < mesh_->polyList[id]->vers.size(); jd++){
                f.append(mesh_->polyList[id]->vers[jd]->verID);
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

        mesh_ = make_shared<PolyMesh<double>>(varList.data_);

        for(int id = 0; id < py::len(vs); id++){
            py::list v = vs[id];
            //py::print(v);
            Vector3 pt;
            for(int kd = 0; kd < 3; kd++){
                pt[kd] = py::float_(v[kd]);
            }
            mesh_->vertexList.push_back(make_shared<VPoint<double>>(pt));
        }

        for(int id = 0; id < py::len(fs); id++)
        {
            py::list f = fs[id];
            //py::print(f);
            pPolygon poly = make_shared<_Polygon<double>>();
            for(int kd = 0; kd < len(f); kd++)
            {
                int vid = py::int_(f[kd]);
                pVertex ver = mesh_->vertexList[vid];
                ver->verID = vid;
                poly->vers.push_back(ver);
            }
            mesh_->polyList.push_back(poly);
        }

        mesh_->removeDuplicatedVertices();
    }

};


#endif //TOPOLITE_PYPOLYMESH_H
