//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphNode.h"
#include "Mesh/PolyMesh.h"
#include "Utility/TopoObject.h"
#include "Utility/PolyPolyBoolean.h"

#include <string>
#include <map>

#include <iostream>
#include <algorithm>
#include <set>
#include <tbb/tbb.h>
#include <cmath>

using pairIJ = std::pair<int, int>;

template<typename Scalar>
class ContactGraph : public TopoObject {
public:
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;

    typedef shared_ptr<ContactGraphEdge<Scalar>> pContactGraphEdge;

    typedef weak_ptr<ContactGraphEdge<Scalar>> wpContactGraphEdge;

    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef weak_ptr<_Polygon<Scalar>> wpPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;


public:

    struct plane_contact {

        Matrix<Scalar, 3, 1> nrm;
        double D;
        int partID;
        int groupID;
        wpPolygon polygon;
        double eps;
    };

    struct plane_contact_compare {
        bool operator()(const plane_contact &A, const plane_contact &B) const {
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

public:
    vector<pContactGraphNode> nodes;
    vector<pContactGraphEdge> edges;
    //automatic generate
    vector<wpContactGraphNode> dynamic_nodes;

private:
    // Class attributes used in constructFromPolyMesh method - Should not be accessed
    vector<pPolyMesh> meshes_input;
    vector<plane_contact> planes;
    vector<pairIJ> planeIJ;
    vector<pContactGraphEdge> planeIJEdges;

public:

    explicit ContactGraph(const shared_ptr<InputVarList> &varList);

    ~ContactGraph();

public:

    bool constructFromPolyMeshes(vector<pPolyMesh> &meshes,
                                 vector<bool> &atBoundary,
                                 double eps = 0.002);

public:

    void normalize_meshes(vector<pPolyMesh> &meshes);

public:

    /*************************************************
    *
    *                  Graph Operation
    *
    *************************************************/

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void finalize();

    void getContactMesh(pPolyMesh &mesh);

private:

    /*************************************************
    *
    *             constructFromPolyMesh methods
    *
    *************************************************/

    double scaleMeshIntoUnitedBox();

    bool createContactPlanes(std::set<plane_contact, plane_contact_compare> &setPlanes, double maxD, double eps);

    void createNodes(vector<bool> &atBoundary);

    void findAllPairsOfPolygonsContact(vector<bool> &atBoundary);

    void addContactEdges();

    void computeContacts();
    /*************************************************
    *
    *           end of scaleMeshIntoUnitedBox methods
    *
    *************************************************/
};

#include "ContactGraph.cpp"

#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
