//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphNode.h"
#include "Mesh/PolyMesh.h"
#include "Utility/TopoObject.h"
#include <string>
#include <map>

using pairIJ = std::pair<int, int>;

using pContactGraphNode = shared_ptr<ContactGraphNode>;
using pContactGraphEdge = shared_ptr<ContactGraphEdge>;
using wpContactGraphEdge = weak_ptr<ContactGraphEdge>;
using wpContactGraphNode = weak_ptr<ContactGraphNode>;

class ContactGraph : TopoObject
{
public:

    vector<shared_ptr<ContactGraphNode>> nodes;
    vector<shared_ptr<ContactGraphEdge>> edges;
    //automatic generate
    vector<weak_ptr<ContactGraphNode>> dynamic_nodes;

public:

    ContactGraph(shared_ptr<InputVarList> varList);

    ~ContactGraph();

public:

    struct plane_contact{
        EigenPoint nrm;
        int nx, ny, nz;
        int D;
        int partID;
        weak_ptr<_Polygon> polygon;
    };

    bool constructFromPolyMeshes(   vector<shared_ptr<PolyMesh>> &meshes,
                                    vector<bool> &atBoundary);

    void roundPlane(shared_ptr<_Polygon> poly, int partID, plane_contact &plane);

    bool equalPlane(const plane_contact &A, const plane_contact &B);

    static bool comparePlane(const plane_contact &A, const plane_contact &B);



public:

    /*************************************************
    *                  Basic Operation
    *************************************************/

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void finalize();
};

#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
