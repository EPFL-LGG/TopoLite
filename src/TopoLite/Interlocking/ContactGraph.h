//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphNode.h"
#include "Mesh/PolyMesh.h"
#include <string>
#include <map>

using pContactGraphNode = shared_ptr<ContactGraphNode>;
using pContactGraphEdge = shared_ptr<ContactGraphEdge>;
using wpContactGraphEdge = weak_ptr<ContactGraphEdge>;
using wpContactGraphNode = weak_ptr<ContactGraphNode>;

class ContactGraph
{
public:

    vector<shared_ptr<ContactGraphNode>> nodes;
    vector<shared_ptr<ContactGraphEdge>> edges;
    //automatic generate
    vector<weak_ptr<ContactGraphNode>> dynamic_nodes;

public:

    ContactGraph();

    ~ContactGraph();

public:

    bool constructFromPolyMeshes(   vector<shared_ptr<PolyMesh>> &meshes,
                                    vector<bool> &atBoundary);

    void roundPlane(const _Polygon* poly);

public:

    /*************************************************
    *                  Basic Operation
    *************************************************/

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void finalize();
};

#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
