//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYCONTACTGRAPH_H
#define TOPOLITE_PYCONTACTGRAPH_H

#include "Interlocking/ContactGraph.h"

#include "PyParamList.h"
#include "PyPolyMesh.h"

#include "igl/writeOBJ.h"
#include "IO/InputVar.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the `_a` literal

class PyContactGraph{
public:

    typedef shared_ptr<ContactGraph<double>> pContactGraph;
    typedef shared_ptr<ContactGraphNode<double>> pContactGraphNode;
    typedef shared_ptr<PolyMesh<double>> pPolyMesh;

public:

    pContactGraph graph;

public:

    PyContactGraph(const vector<PyPolyMesh> &pyPolymeshes, float contact_eps)
    {
        if(pyPolymeshes.empty() || pyPolymeshes.front().mesh_ == nullptr) return;
        graph = make_shared<ContactGraph<double>>(pyPolymeshes.front().mesh_->getVarList());

        vector<pPolyMesh> meshes;
        vector<bool> atBoundary;

        for(size_t id = 0; id < pyPolymeshes.size(); id++)
        {
            meshes.push_back(pyPolymeshes[id].mesh_);
            atBoundary.push_back(pyPolymeshes[id].atBoundary_);
        }

        graph->buildFromMeshes(meshes, atBoundary, contact_eps);
        graph->finalize();
    }

public:

    PyPolyMesh getContacts()
    {
        PyPolyMesh pyPolyMesh;
        if (graph)
        {
            pyPolyMesh.atBoundary_ = false;
            graph->getContactMesh(pyPolyMesh.mesh_);
        }
        return pyPolyMesh;
    }

    bool mergeParts(int partI, int partJ){
        if(partI >= 0 && partI < graph->nodes.size() && partJ >= 0 && partJ < graph->nodes.size()){
            pContactGraphNode nodeI, nodeJ;
            nodeI = graph->nodes[partI];
            nodeJ = graph->nodes[partJ];
            graph->mergeNode(nodeI, nodeJ);
            return false;
        }
        return false;
    }

    int numContacts()
    {
        if(graph)
            return graph->edges.size();
        return 0;
    }
};

#endif //TOPOLITE_PYCONTACTGRAPH_H
