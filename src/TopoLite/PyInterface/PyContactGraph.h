//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYCONTACTGRAPH_H
#define TOPOLITE_PYCONTACTGRAPH_H

#include "igl/writeOBJ.h"
#include "Interlocking/ContactGraph.h"
#include "PyTopoCreator.h"
#include "IO/InputVar.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Mesh/MeshConverter.h"
namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyContactGraph{

public:

    shared_ptr<ContactGraph> graph;

public:

    PyContactGraph()
    {

    }

    PyContactGraph(const PyTopoCreator &instance)
    {
        PyContactGraph();
        const XMLData *data = instance.data.get();
        shared_ptr<Struc> struc = data->strucCreator->struc;

        graph = make_shared<ContactGraph>(data->varList);

        vector<shared_ptr<PolyMesh>> meshes;
        vector<bool> atBoundary;

//        std::cout << struc->partList.size() << std::endl;
        for(int partID = 0; partID < struc->partList.size(); partID++)
        {
            pPart part = struc->partList[partID];
            meshes.push_back(part->polyMesh);
            atBoundary.push_back(part->atBoundary);
        }
        graph->constructFromPolyMeshes(meshes, atBoundary);

        graph->finalize();
    }

    PyContactGraph(py::list pyMeshes, py::list pyAtBoundary, bool triMesh)
    {
        PyContactGraph();
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());
        graph = make_shared<ContactGraph>(varList);

        vector<pPolyMesh> meshes;
        vector<bool> atBoundary;
        for(int id = 0; id < py::len(pyMeshes); id++)
        {
            py::object pyMesh = pyMeshes[id];
            pPolyMesh polyMesh;
            Compas2PolyMesh(pyMesh, polyMesh, varList);
            meshes.push_back(polyMesh);
            atBoundary.push_back(py::bool_(pyAtBoundary[id]));
        }

        if(triMesh)
            graph->mergeFacesPolyMesh(meshes);

        graph->constructFromPolyMeshes(meshes, atBoundary);
        graph->finalize();
    }

public:

    py::list mergeFaces(py::list &pyMeshes)
    {
        vector<pPolyMesh> meshes;
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());

        for(int id = 0; id < py::len(pyMeshes); id++)
        {
            py::object pyMesh = pyMeshes[id];
            pPolyMesh polyMesh;
            Compas2PolyMesh(pyMesh, polyMesh, varList);
            meshes.push_back(polyMesh);
        }

        shared_ptr<ContactGraph> tmp_graph = make_shared<ContactGraph>(varList);
        tmp_graph->mergeFacesPolyMesh(meshes);

        pyMeshes = py::list();

        for(int id = 0; id < meshes.size(); id++)
        {
            py::object pyMesh;
            PolyMesh2Compas(pyMesh, meshes[id]);
            pyMeshes.append(pyMesh);
        }

        return pyMeshes;
    }

    py::object getContacts()
    {
        if (graph)
        {
            pPolyMesh contact_mesh = make_shared<PolyMesh>(graph->getVarList());
            graph->getContactMesh(contact_mesh);

            if (contact_mesh)
            {
                contact_mesh->UpdateVertices();
                py::object compas_mesh;
                PolyMesh2Compas(compas_mesh, contact_mesh);
                return compas_mesh;
            }
        }
        return py::object();
    }

    void PolyMesh2Compas(py::object &pyMesh, const pPolyMesh &polyMesh)
    {
        py::list vs;
        py::list fs;

        for(int id = 0; id < polyMesh->vertexList.size(); id++){
            py::list v;
            v.append(polyMesh->vertexList[id].x);
            v.append(polyMesh->vertexList[id].y);
            v.append(polyMesh->vertexList[id].z);
            vs.append(v);
        }
        for(int id = 0; id < polyMesh->polyList.size(); id++){
            py::list f;
            for(int jd = 0; jd < polyMesh->polyList[id]->verIDs.size(); jd++){
                f.append(polyMesh->polyList[id]->verIDs[jd]);
            }
            fs.append(f);
        }
        py::object module_compas_datastructures;
        py::object module_compas_mesh;
        module_compas_datastructures = py::module::import("compas.datastructures");
        module_compas_mesh = module_compas_datastructures.attr("Mesh");
        pyMesh = module_compas_mesh.attr("from_vertices_and_faces")(vs, fs);
    }

    void Compas2PolyMesh(const py::object &compasMesh, pPolyMesh &polyMesh, shared_ptr<InputVarList> varList)
    {
        py::list vfs = compasMesh.attr("to_vertices_and_faces")();
        py::list vs = vfs[0];
        py::list fs = vfs[1];

        polyMesh = make_shared<PolyMesh>(varList);
        polyMesh->vertexList.resize(py::len(vs));

        for(int id = 0; id < py::len(vs); id++){
            py::list v = vs[id];
            //py::print(v);
            Vector3f pt;
            for(int kd = 0; kd < 3; kd++){
                pt[kd] = py::float_(v[kd]);
            }
            polyMesh->vertexList[id] = pt;
        }

        for(int id = 0; id < py::len(fs); id++)
        {
            py::list f = fs[id];
            //py::print(f);
            shared_ptr<_Polygon> poly = make_shared<_Polygon>();
            for(int kd = 0; kd < len(f); kd++){
                int vid = py::int_(f[kd]);
                poly->verIDs.push_back(vid);
                poly->vers.push_back(polyMesh->vertexList[vid]);
            }
            polyMesh->polyList.push_back(poly);
        }

        polyMesh->UpdateVertices();
    }

public:

    int numContacts()
    {
        if(graph)
            return graph->edges.size();
        return 0;
    }
};

#endif //TOPOLITE_PYCONTACTGRAPH_H
