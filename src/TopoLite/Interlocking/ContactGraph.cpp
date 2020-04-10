//
// Created by ziqwang on 14.01.19.
//


#include "ContactGraph.h"
#include "Utility/ConvexHull2D.h"

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
 * @brief build the graph from a list of polygonal meshes
 * @tparam Scalar
 * @param meshes
 * @param atBoundary
 * @param eps
 * @param convexhull : merges faces which on a same plane by its convexhull
 * @return
 */
template<typename Scalar>
bool ContactGraph<Scalar>::buildFromMeshes(vector<pPolyMesh> &input_meshes,
                                           vector<bool> &atBoundary,
                                           Scalar eps,
                                           bool convexhull) {

    // [1] - Scale meshes_input into a united box
    meshes_input = input_meshes;


    // [2] - cluster the faces of input mesh which are on a same plane
    contact_faces.clear();
    if(!clusterFacesofInputMeshes(eps))
        return false;

    tbb::parallel_sort(contact_faces.begin(), contact_faces.end(), [&](const polygonal_face &A, const polygonal_face &B) {
        return A.groupID < B.groupID;
    });

    // [3] - find all pairs of potential face contacts
    contact_pairs.clear();
    listPotentialContacts(atBoundary);

    // [4] - compute the contacts
    contact_graphedges.clear();
    computeContacts();  // note: this function is vectorized with TBB

    // [5] - add nodes into the graph
    nodes.clear();
    buildNodes(atBoundary);

    // [6] - add faces into graph
    edges.clear();
    buildEdges();

    // [7] - assign node ID
    finalize();
    contact_edges = edges;

    // [8] - simplify contacts
    if(convexhull) convexhullEdges();

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
 * @brief merging nodeA and nodeB. they should move together.
 * @tparam Scalar
 * @param _nodeA,_nodeB
 */
template<typename Scalar>
void ContactGraph<Scalar>::mergeNode(ContactGraph::pContactGraphNode _nodeA, ContactGraph::pContactGraphNode _nodeB) {
    merged_nodes.push_back({_nodeA, _nodeB});
}

/**
 * @brief Assign nodeID to the graph. Must be called after updating the graph
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
 * @brief return the all contact polygons as a mesh
 * @tparam Scalar
 * @param mesh
 */
template<typename Scalar>
void ContactGraph<Scalar>::getContactMesh(pPolyMesh &mesh) {
    mesh.reset();
    mesh = make_shared<PolyMesh<Scalar>>(getVarList());
    for (pContactGraphEdge edge: contact_edges) {
        for (pPolygon poly: edge->polygons) {
            pPolygon copy_poly = make_shared<_Polygon<Scalar>>(*poly);
            if(edge->partIDA > edge->partIDB){
                copy_poly->reverseVertices();
            }
            mesh->polyList.push_back(copy_poly);
        }
    }

    if (mesh) mesh->removeDuplicatedVertices();

//    mesh->ScaleMesh(1.0 / normalized_scale);
//    mesh->TranslateMesh(-normalized_trans);
}




/*************************************************
*
*             constructFromPolyMesh methods
*
*************************************************/

/**
 * @brief extract all polygonal faces from the meshes_input and cluster the faces which are on a same plane.
 * @tparam Scalar
 * @param eps
 * @return
 */
template<typename Scalar>
bool ContactGraph<Scalar>::clusterFacesofInputMeshes(Scalar eps)
{
    std::set<polygonal_face, plane_contact_compare> planes_set;

    int groupID = 0;
    size_t msize = meshes_input.size();
    for (size_t id = 0; id < msize; id++)
    {
        pPolyMesh poly = meshes_input[id];
        if (poly == nullptr)
            return false;

        for (pPolygon face : poly->polyList)
        {
            if (face->vers.size() < 3)
                continue;

            // 1.1) construct plane
            polygonal_face plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = nrm;

            plane.D = nrm.dot(center);
            plane.partID = id;
            plane.polygon = face;
            plane.eps = eps;

            // 1.2) find groupID
            typename std::set<polygonal_face, plane_contact_compare>::iterator find_it = planes_set.end();
            for (int reverse = -1; reverse <= 1; reverse += 2)
            {
                polygonal_face tmp_plane = plane;
                tmp_plane.nrm *= reverse;
                tmp_plane.D *= reverse;

                find_it = planes_set.find(tmp_plane);
                if (find_it != planes_set.end())
                {
                    plane.groupID = (*find_it).groupID;
                    break;
                }
            }

            if (find_it == planes_set.end())
            {
                plane.groupID = groupID++;
                planes_set.insert(plane);
            }

            contact_faces.push_back(plane);
        }
    }
    return true;
}

/**
 * @brief find all pairs of faces which are on a same plane. Their normals have to be on opposite directions
 * @tparam Scalar
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraph<Scalar>::listPotentialContacts(vector<bool> &atBoundary)
{
    size_t sta = 0, end = 0;
    while (sta < contact_faces.size())
    {
        for (end = sta + 1; end < contact_faces.size(); end++)
        {
            if (contact_faces[sta].groupID != contact_faces[end].groupID)
            {
                break;
            }
        }
        if (end - sta > 1)
        {
            for (int id = sta; id < end; id++) {
                for (int jd = id + 1; jd < end; jd++)
                {
                    int partI = contact_faces[id].partID;
                    int partJ = contact_faces[jd].partID;

                    Vector3 nrmI = contact_faces[id].nrm.normalized();
                    Vector3 nrmJ = contact_faces[jd].nrm.normalized();

                    if (partI != partJ
                        && std::abs(nrmI.dot(nrmJ) + 1) < FLOAT_ERROR_LARGE
                        && (!atBoundary[partI] || !atBoundary[partJ])){
                        contact_pairs.push_back(partI < partJ ? pairIJ(id, jd) : pairIJ(jd, id));
                    }

                }
            }
        }
        sta = end;
    }
}

/**
 * @brief Parallel compute contacts.
 *        Though faces in the 'contact_pairs' are on the same plane,
 *        checking whether they are overlap needs the 2D poly-poly intersection tool.
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraph<Scalar>::computeContacts()
{
    size_t psize = contact_pairs.size();
    contact_graphedges.resize(psize);
    //for (size_t id = 0; id < psize; ++id) // Sequential loop
    tbb::parallel_for(tbb::blocked_range<size_t>(0, psize), [&](const tbb::blocked_range<size_t> &r)
    {
        for (size_t id = r.begin(); id != r.end(); ++id) {


            int planeI = contact_pairs[id].first;
            int planeJ = contact_pairs[id].second;

            vector<Vector3> polyI = contact_faces[planeI].polygon.lock()->getVertices();
            vector<Vector3> polyJ = contact_faces[planeJ].polygon.lock()->getVertices();

            vector<vector<Vector3>> contactPtLists;

            PolyPolyBoolean<Scalar> ppIntersec(getVarList());

            ppIntersec.computePolygonsIntersection(polyI, polyJ, contactPtLists);
            if (contactPtLists.empty())
            {
                continue;
            }

            vector<pPolygon> contactPolys;
            for (vector<Vector3> contactPtList: contactPtLists)
            {
                if (contactPtList.size() >= 3) {
                    pPolygon contactPoly = make_shared<_Polygon<Scalar>>();
                    contactPoly->setVertices(contactPtList);
                    contactPolys.push_back(contactPoly);
                }
            }

            if (!contactPolys.empty())
            {
                contact_graphedges[id] = make_shared<ContactGraphEdge<Scalar>>(contactPolys, contact_faces[planeI].nrm);
            }
        }
    });
}

/**
 * @brief add each part as a node into the graph
 * @tparam Scalar
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraph<Scalar>::buildNodes(vector<bool> &atBoundary)
{
    for (size_t id = 0; id < meshes_input.size(); id++) {
        Vector3 centroid = meshes_input[id]->centroid();
        Scalar volume = meshes_input[id]->volume();
        pContactGraphNode node = make_shared<ContactGraphNode<Scalar>>(atBoundary[id], centroid, centroid, volume);
        addNode(node);
    }
}

/**
 * @brief for all pairs of potential contact faces,
 *        if their contact_graphedges is not empty, then we add this edge into the graph
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraph<Scalar>::buildEdges() {

    for (size_t id = 0; id < contact_pairs.size(); id++)
    {
        pContactGraphEdge edge = contact_graphedges[id];
        if (edge != nullptr) {
            int planeI = contact_pairs[id].first;
            int planeJ = contact_pairs[id].second;
            int partI = contact_faces[planeI].partID;
            int partJ = contact_faces[planeJ].partID;
            addContact(nodes[partI], nodes[partJ], edge);
        }
    }
}

/**
 * @brief simplified the contact between two nodes, only compute their convex-hull
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraph<Scalar>::convexhullEdges()
{
    std::sort(contact_edges.begin(), contact_edges.end(), [&](pContactGraphEdge e0, pContactGraphEdge e1){
        if(e0->partIDA < e1->partIDA)
            return true;
        if(e0->partIDA > e1->partIDA)
            return false;

        if(e0->partIDB < e1->partIDB)
            return true;
        if(e0->partIDB > e1->partIDB)
            return false;

        if (e0->normal[0] - e1->normal[0] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[0] - e1->normal[0] > FLOAT_ERROR_LARGE)
            return false;

        if (e0->normal[1] - e1->normal[1] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[1] - e1->normal[1] > FLOAT_ERROR_LARGE)
            return false;

        if (e0->normal[2] - e1->normal[2] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[2] - e1->normal[2] > FLOAT_ERROR_LARGE)
            return false;

        return false;
    });


    edges.clear();
    size_t sta = 0, end = 0;

    while (sta < contact_edges.size()) {
        for (end = sta + 1; end < contact_edges.size(); end++) {
            if(!contact_edges[sta]->check_on_same_plane(contact_edges[end])){
                break;
            }
        }

        pContactGraphEdge edge;
        if(end > sta + 1)
        {
            vector<Vector3> corner_points;
            for(int id = sta; id < end; id++)
            {
                for(pPolygon poly: contact_edges[id]->polygons){
                    vector<Vector3> poly_points = poly->getVertices();
                    corner_points.insert(corner_points.end(), poly_points.begin(), poly_points.end());
                }
            }

            Vector3 normal = contact_edges[sta]->normal, x_axis, y_axis;

            vector<Vector3> convexhull_points;
            ConvexHull2D<double> convexhull_solver;
            convexhull_solver.compute(corner_points, normal, convexhull_points);

            pPolygon convexhull_poly = make_shared<_Polygon<Scalar>>();
            convexhull_poly->setVertices(convexhull_points);

            edge = make_shared<ContactGraphEdge<Scalar>>(convexhull_poly , contact_edges[sta]->normal);
        }
        else{
            edge = make_shared<ContactGraphEdge<Scalar>>(contact_edges[sta]->polygons, contact_edges[sta]->normal);
        }

        edge->partIDA = contact_edges[sta]->partIDA;
        edge->partIDB = contact_edges[sta]->partIDB;
        edges.push_back(edge);
        sta = end;
    }

}
