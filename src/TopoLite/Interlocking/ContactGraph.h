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
using pPolyMesh = shared_ptr<PolyMesh>;
struct plane_contact
{
    EigenPoint nrm;
    double D;
    int partID;
    int groupID;
    weak_ptr<_Polygon> polygon;
    double eps;
};

struct plane_contact_compare
{
    bool operator()(const plane_contact& A, const plane_contact& B) const
    {
        double eps = A.eps / 2;

        if (A.nrm[0] - B.nrm[0] < -eps)
            return true;
        if (A.nrm[0] - B.nrm[0] > eps)
            return false;

        if (A.nrm[1] - B.nrm[1] < -eps)
            return true;
        if (A.nrm[1] - B.nrm[1] > eps)
            return false;

        if (A.nrm[2] - B.nrm[2] < -eps)
            return true;
        if (A.nrm[2] - B.nrm[2] > eps)
            return false;

        if (A.D - B.D < -eps)
            return true;
        if (A.D - B.D > eps)
            return false;

        return false;
    }
};

class ContactGraph : public TopoObject
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

    bool constructFromPolyMeshes(   vector<shared_ptr<PolyMesh>> &meshes,
                                    vector<bool> &atBoundary,
                                    double eps = 0.002);

    void mergeFacesPolyMesh(vector<shared_ptr<PolyMesh>> &meshes, double eps = 1e-4);

public:

    /*************************************************
    *                  Basic Operation
    *************************************************/

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void finalize();

public:

    void getContactMesh(pPolyMesh &mesh);
};

#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
