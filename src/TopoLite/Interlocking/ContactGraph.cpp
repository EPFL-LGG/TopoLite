//
// Created by ziqwang on 14.01.19.
//


#include "ContactGraph.h"

/*************************************************
*
*                  Basic Operation
*
*************************************************/

/**
 * @brief Class constructor
 * @tparam Scalar
 * @param varList
 */
template<typename Scalar>
ContactGraph<Scalar>::ContactGraph(const shared_ptr<InputVarList> &varList):TopoObject(varList) {}

/**
 * @brief Class destructor
 * @tparam Scalar
 */
template<typename Scalar>
ContactGraph<Scalar>::~ContactGraph() {
    nodes.clear();
    edges.clear();
    dynamic_nodes.clear();
}


/**
 * @brief
 * @tparam Scalar
 * @param meshes
 * @param atBoundary
 * @param eps
 * @return
 */
template<typename Scalar>
bool ContactGraph<Scalar>::constructFromPolyMeshes(vector<pPolyMesh> &input_meshes,
                                                   vector<bool> &atBoundary,
                                                   double eps) {

    // [1] - Scale meshes_input into a united box
    meshes_input = input_meshes;

    auto maxD = scaleMeshIntoUnitedBox();

    if (maxD == -1.0)
        return false;

    // [2] - Create contact planes

    std::set<plane_contact, plane_contact_compare> setPlanes;

    bool res = createContactPlanes( setPlanes, maxD, eps);

    if (!res)
        return false;

    std::sort(planes.begin(), planes.end(), [&](const plane_contact &A, const plane_contact &B) {
        return A.groupID < B.groupID;
    });

    //

    createNodes( atBoundary);

    findAllPairsOfPolygonsContact( atBoundary);

    planeIJEdges.resize(planeIJ.size());

    computeContacts();  // note: this function is vectorized with TBB

    addContactEdges();

    return true;
}

/*************************************************
*
*                  Graph Operation
*
*************************************************/
/**
 * @brief
 * @tparam Scalar
 * @param _node
 */
template<typename Scalar>
void ContactGraph<Scalar>::addNode(pContactGraphNode _node) {
    _node->staticID = nodes.size();

    nodes.push_back(_node);
}

/**
 * @brief
 * @tparam Scalar
 * @param _nodeA
 * @param _nodeB
 * @param _edge
 */
template<typename Scalar>
void ContactGraph<Scalar>::addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge) {

    if (_nodeA->isBoundary && _nodeB->isBoundary)
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
}

/**
 * @brief
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraph<Scalar>::finalize() {
    int dynamicID = 0;
    dynamic_nodes.clear();
    for (pContactGraphNode node : nodes) {
        if (!node->isBoundary) {
            node->dynamicID = dynamicID++;
            dynamic_nodes.push_back(node);
        } else {
            node->dynamicID = -1;
        }
    }
}

/**
 * @brief
 * @tparam Scalar
 * @param mesh
 */
template<typename Scalar>
void ContactGraph<Scalar>::getContactMesh(pPolyMesh &mesh) {
    mesh.reset();
    mesh = make_shared<PolyMesh<Scalar>>(getVarList());
    for (pContactGraphEdge edge: edges) {
        for (pPolygon poly: edge->polygons) {
            mesh->polyList.push_back(poly);
        }
    }

    if (mesh) mesh->removeDuplicatedVertices();

//    mesh->ScaleMesh(1.0 / normalized_scale);
//    mesh->TranslateMesh(-normalized_trans);
}


/**
 * @brief
 * @tparam Scalar
 * @param meshes
 */
// TODO: implement or clean this unimplemented function
template<typename Scalar>
void ContactGraph<Scalar>::normalize_meshes(vector<pPolyMesh> &meshes) {}

/*************************************************
*
*             constructFromPolyMesh methods
*
*************************************************/

/**
 * @brief Scale the Mesh into a united box
 * @tparam Scalar
 * @param meshes the vector containing the meshes_input to scale
 * @return maxD
 */
template<typename Scalar>
double ContactGraph<Scalar>::scaleMeshIntoUnitedBox() {
    double maxD = 1;
    size_t msize = meshes_input.size();
    for (size_t id = 0; id < msize; id++) {
        pPolyMesh poly = meshes_input[id];
        if (poly == nullptr)
            return -1.0;
        for (pPolygon face : poly->polyList) {
            // 1.1) construct plane
            plane_contact plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = nrm;

            plane.D = nrm.dot(center);
            maxD = (std::max)(maxD, (double) std::abs(plane.D));
        }
    }
    return maxD;
}

/**
 * @brief @Ziqi Please comment this a bit
 * @tparam Scalar
 * @param meshes
 * @param planes
 * @param setPlanes
 * @param maxD
 * @param eps
 * @return
 */
template<typename Scalar>
bool ContactGraph<Scalar>::createContactPlanes(std::set<plane_contact, plane_contact_compare> &setPlanes,
                                               double maxD, double eps) {

    int groupID = 0;
    size_t msize = meshes_input.size();
    for (size_t id = 0; id < msize; id++) {
        pPolyMesh poly = meshes_input[id];
        if (poly == nullptr)
            return false;
        for (pPolygon face : poly->polyList) {
            if (face->vers.size() < 3)
                continue;

            // 1.1) construct plane
            plane_contact plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = nrm;

            plane.D = nrm.dot(center) / maxD;
            plane.partID = id;
            plane.polygon = face;
            plane.eps = eps;

            // 1.2) find groupID
            typename std::set<plane_contact, plane_contact_compare>::iterator find_it = setPlanes.end();
            for (int reverse = -1; reverse <= 1; reverse += 2) {
                plane_contact tmp_plane = plane;
                tmp_plane.nrm *= reverse;
                tmp_plane.D *= reverse;
                find_it = setPlanes.find(tmp_plane);
                if (find_it != setPlanes.end()) {
                    plane.groupID = (*find_it).groupID;
                    break;
                }
            }

            if (find_it == setPlanes.end()) {
                plane.groupID = groupID++;
                setPlanes.insert(plane);
            }

            planes.push_back(plane);
        }
    }
    return true;
}

/**
 * @brief
 * @tparam Scalar
 * @param meshes
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraph<Scalar>::createNodes(vector<bool> &atBoundary) {
    for (size_t id = 0; id < meshes_input.size(); id++) {
        Vector3 centroid = meshes_input[id]->centroid();
        Scalar volume = meshes_input[id]->volume();
        pContactGraphNode node = make_shared<ContactGraphNode<Scalar>>(atBoundary[id], centroid, centroid, volume);
        addNode(node);
    }
}

/**
 * @brief @Ziqi same here please
 * @tparam Scalar
 * @param planeIJ
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraph<Scalar>::findAllPairsOfPolygonsContact(vector<bool> &atBoundary) {
    int sta = 0, end = 0;

    while (sta < planes.size()) {
        for (end = sta + 1; end < planes.size(); end++) {
            if (planes[sta].groupID != planes[end].groupID) {
                break;
            }
        }
        if (end - sta > 1) {
            for (int id = sta; id < end; id++) {
                for (int jd = id + 1; jd < end; jd++) {
                    int partI = planes[id].partID;
                    int partJ = planes[jd].partID;

                    Vector3 nrmI = planes[id].nrm.normalized();
                    Vector3 nrmJ = planes[jd].nrm.normalized();

                    if (partI != partJ
                        && std::abs(nrmI.dot(nrmJ) + 1) < FLOAT_ERROR_LARGE
                        && (!atBoundary[partI] || !atBoundary[partJ]))
                        planeIJ.push_back(pairIJ(id, jd));
                }
            }
        }
        sta = end;
    }
}

/**
 * @brief
 * @tparam Scalar
 * @param planeIJ
 * @param planeIJEdges
 */
template<typename Scalar>
void ContactGraph<Scalar>::addContactEdges() {

    for (size_t id = 0; id < planeIJ.size(); id++) {
        pContactGraphEdge edge = planeIJEdges[id];
        if (edge != nullptr) {
            int planeI = planeIJ[id].first;
            int planeJ = planeIJ[id].second;
            int partI = planes[planeI].partID;
            int partJ = planes[planeJ].partID;
            addContact(nodes[partI], nodes[partJ], edge);
        }
    }
}

/**
 * @brief Parallel compute contacts
 * @tparam Scalar
 * @param planeIJ
 * @param planeIJEdges
 */
template<typename Scalar>
void ContactGraph<Scalar>::computeContacts() {

    size_t psize = planeIJ.size();
    // for (size_t id = 0; id != planeIJ.size(); ++id) // Sequential loop
    tbb::parallel_for(tbb::blocked_range<size_t>(0, psize), [&](const tbb::blocked_range<size_t> &r) {
        for (size_t id = r.begin(); id != r.end(); ++id) {

            int planeI = planeIJ[id].first;
            int planeJ = planeIJ[id].second;

            vector<Vector3> polyI = planes[planeI].polygon.lock()->getVertices();
            vector<Vector3> polyJ = planes[planeJ].polygon.lock()->getVertices();
            vector<vector<Vector3>> contactPtLists;

            PolyPolyBoolean<Scalar> ppIntersec(getVarList());

            ppIntersec.computePolygonsIntersection(polyI, polyJ, contactPtLists);
            if (contactPtLists.empty())
                continue;

            vector<pPolygon> contactPolys;
            for (vector<Vector3> contactPtList: contactPtLists) {
                if (contactPtList.size() >= 3) {
                    pPolygon contactPoly = make_shared<_Polygon<Scalar>>();
                    contactPoly->setVertices(contactPtList);
                    contactPolys.push_back(contactPoly);
                }
            }

            if (!contactPolys.empty()) {
                planeIJEdges[id] = make_shared<ContactGraphEdge<Scalar>>(contactPolys, planes[planeI].nrm);
            }
        }
    });
}
