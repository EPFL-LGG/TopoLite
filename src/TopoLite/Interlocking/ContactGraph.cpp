//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"
#include <iostream>
#include <algorithm>

/*************************************************
*
*                  Basic Operation
*
*************************************************/
using pPolyMesh = shared_ptr<PolyMesh>;
ContactGraph::ContactGraph()
{

}

ContactGraph::~ContactGraph()
{
    nodes.clear();
    edges.clear();
}

bool ContactGraph::constructFromPolyMeshes(vector<shared_ptr<PolyMesh>> &meshes,
                                           vector<bool> &atBoundary)
{
    const float normal2int_scalefactor = 100000;

    struct plane_contact{
        int nx, ny, nz;
        int D;
        int partID;
        weak_ptr<_Polygon> polygon;
    };

    vector<plane_contact> planes;
    for(int id = 0; id < meshes.size(); id++){
        pPolyMesh poly = meshes[id];
        if(poly == nullptr) return false;
        for(shared_ptr<_Polygon> face: poly->polyList){

            Vector3f nrm = face->ComputeNormal();
            Vector3f center = face->ComputeCenter();

            int reverse = 1;
            if(nrm[0] < 0) reverse = -1;
            if(nrm[0] == 0 && nrm[1] < 0) reverse = -1;
            if(nrm[0] == 0 && nrm[1] == 0 && nrm[2] < 0) reverse = -1;
            nrm *= reverse;

            plane_contact plane;
            plane.nx = (int)(nrm[0] * normal2int_scalefactor);
            plane.ny = (int)(nrm[1] * normal2int_scalefactor);
            plane.nz = (int)(nrm[2] * normal2int_scalefactor);
            plane.D = (int)std::floor((nrm ^ center) * normal2int_scalefactor);
            plane.partID = id;
            plane.polygon = face;

            planes.push_back(plane);
        }
    }

    std::sort(planes.begin(), planes.end(),
            [](const plane_contact &A, const plane_contact &B){
            if(A.nx < B.nx) return true;
            if(A.nx > B.nx) return false;
            if(A.ny < B.ny) return true;
            if(A.ny > B.ny) return false;
            if(A.nz < B.nz) return true;
            if(A.nz > B.nz) return false;
            if(A.D < B.D) return true;
            if(A.D > B.D) return false;
            return false;
    });

    for(int id = 0; id < planes.size(); id++){
        std::cout << planes[id].nx
           << " " << planes[id].ny
           << " " << planes[id].nz
           << " " << planes[id].D << std::endl;
    }

    return true;
}

/*************************************************
*
*                  Graph Operation
*
*************************************************/

void ContactGraph::addNode(shared_ptr<ContactGraphNode> _node)
{
    _node->staticID = nodes.size();

    nodes.push_back(_node);

    return;
}

void ContactGraph::addContact(shared_ptr<ContactGraphNode> _nodeA, shared_ptr<ContactGraphNode> _nodeB, shared_ptr<ContactGraphEdge> _edge)
{

    if(_nodeA->isBoundary && _nodeB->isBoundary)
        return;

    _edge->partIDA = _nodeA->staticID;
    _edge->partIDB = _nodeB->staticID;

    edges.push_back(_edge);

    pair<wpContactGraphNode, wpContactGraphEdge> contactNeighbor;

    contactNeighbor.first = _nodeB;
    contactNeighbor.second = _edge;
    _nodeA->neighbors.push_back(contactNeighbor);

    contactNeighbor.first = _nodeA;
    contactNeighbor.second = _edge;
    _nodeB->neighbors.push_back(contactNeighbor);

    return;
}

void ContactGraph::finalize()
{
    int dynamicID = 0;
    dynamic_nodes.clear();
    for(shared_ptr<ContactGraphNode> node: nodes){
        if(!node->isBoundary){
            node->dynamicID = dynamicID ++;
            dynamic_nodes.push_back(node);
        }
        else{
            node->dynamicID = -1;
        }
    }
}

