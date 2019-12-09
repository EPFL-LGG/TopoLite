//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"
#include <iostream>

double ContactGraph::SlopeSearchRegion::minimum_possible_tiltAngle()
{
    va /= va.norm();
    vb /= vb.norm();
    EigenPoint VA = va * std::tan(ta / 180 * M_PI);
    EigenPoint VB = vb * std::tan(tb / 180 * M_PI);

    double d11, d12, d22;
    d11 = VA.dot(VA);
    d12 = VB.dot(VA);
    d22 = VB.dot(VB);

    mu_ = (d12 - d22) / (2*d12 - d11 - d22);
    if(mu_ <= 0 || mu_ >= 1)
    {
        minimum_ = std::min(ta, tb);
    }
    else{
        vc_ = VA * mu_ + VB * (1 - mu_);
        minimum_ = std::atan(vc_.norm()) / M_PI * 180;
        vc_ /= vc_.norm();
    }
    return minimum_;
}

/*************************************************
*
*                  Basic Operation
*
*************************************************/

ContactGraph::ContactGraph(shared_ptr<InputVarList> varList) : TopoObject(varList)
{
    minimum_slope = 0;
    translation.setZero();
}


ContactGraph::~ContactGraph()
{
    nodes.clear();
    edges.clear();
}


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

void ContactGraph::initialize()
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


/*************************************************
*
*               Property Verification
*
*************************************************/

void ContactGraph::computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: edges) {

        int iA = nodes[edge->partIDA]->dynamicID;

        int iB = nodes[edge->partIDB]->dynamicID;

        for (EigenPoint nrm : edge->normals) {
            if (iA != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iA, -nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 1, -nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iA + 2, -nrm[2]));
            }

            if (iB != -1) {
                tri.push_back(EigenTriple(rowID, 3 * iB, nrm[0]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 1, nrm[1]));
                tri.push_back(EigenTriple(rowID, 3 * iB + 2, nrm[2]));
            }
            rowID++;
        }
    }
    size = Eigen::Vector2i(rowID, 3 * dynamic_nodes.size());
}

void ContactGraph::computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: edges)
    {
        shared_ptr<ContactGraphNode> nodeA = nodes[edge->partIDA];
        shared_ptr<ContactGraphNode> nodeB = nodes[edge->partIDB];

        int iA = nodeA->dynamicID;
        int iB = nodeB->dynamicID;

        EigenPoint ctA = nodeA->centroid;
        EigenPoint ctB = nodeB->centroid;

        for(int id = 0; id < edge->normals.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            EigenPoint nrm = edge->normals[id];
            for(EigenPoint pt: poly.points)
            {
                if (iA != -1)
                {
                    EigenPoint mt = (pt - ctA).cross(nrm);
                    tri.push_back(EigenTriple(rowID, 6 * iA, -nrm[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 1, -nrm[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 2, -nrm[2]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 3, -mt[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 4, -mt[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 5, -mt[2]));
                }

                if (iB != -1) {
                    EigenPoint mt = (pt - ctB).cross(nrm);
                    tri.push_back(EigenTriple(rowID, 6 * iB, nrm[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 1, nrm[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 2, nrm[2]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 3, mt[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 4, mt[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iB + 5, mt[2]));
                }
                rowID++;
            }
        }
    }
    size = Eigen::Vector2i(rowID, 6 * dynamic_nodes.size());
}

void ContactGraph::get_force_from_norm_fric(Eigen::Vector3d n,       Eigen::Vector3d u,       Eigen::Vector3d v,
                                            Eigen::RowVector2d &fkx, Eigen::RowVector2d &fky, Eigen::RowVector2d &fkz)
{
    Eigen::Vector3d x_comp(1, 0 ,0);
    Eigen::Vector3d y_comp(0, 1 ,0);
    Eigen::Vector3d z_comp(0, 0 ,1);
    fkx = Eigen::RowVector2d(n.dot(x_comp), -n.dot(x_comp));
    fky = Eigen::RowVector2d(n.dot(y_comp), -n.dot(y_comp));
    fkz = Eigen::RowVector2d(n.dot(z_comp), -n.dot(z_comp));
    return;
}

void ContactGraph::get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                            Eigen::RowVector4d    &fkx, Eigen::RowVector4d &fky, Eigen::RowVector4d &fkz){
    Eigen::Vector3d x_comp(1, 0 ,0);
    Eigen::Vector3d y_comp(0, 1 ,0);
    Eigen::Vector3d z_comp(0, 0 ,1);
    fkx = Eigen::RowVector4d(n.dot(x_comp), -n.dot(x_comp), u.dot(x_comp), v.dot(x_comp));
    fky = Eigen::RowVector4d(n.dot(y_comp), -n.dot(y_comp), u.dot(y_comp), v.dot(y_comp));
    fkz = Eigen::RowVector4d(n.dot(z_comp), -n.dot(z_comp), u.dot(z_comp), v.dot(z_comp));
}

void ContactGraph::get_moment_from_norm_fric_vertex(Eigen::Vector3d n, Eigen::Vector3d u, Eigen::Vector3d v,
                                                    Eigen::Vector3d r, Eigen::RowVector2d &mx, Eigen::RowVector2d &my,
                                                    Eigen::RowVector2d &mz)
{
    Eigen::Vector3d x_comp(1, 0 ,0);
    Eigen::Vector3d y_comp(0, 1 ,0);
    Eigen::Vector3d z_comp(0, 0 ,1);
//    Eigen::Vector3d m_normal = n.cross(r);
//    Eigen::Vector3d m_u_fric = u.cross(r);
//    Eigen::Vector3d m_v_fric = v.cross(r);
    Eigen::Vector3d m_normal = -r.cross(n);
    Eigen::Vector3d m_u_fric = -r.cross(u);
    Eigen::Vector3d m_v_fric = -r.cross(v);

    mx = Eigen::RowVector2d(m_normal.dot(x_comp), -m_normal.dot(x_comp));
    my = Eigen::RowVector2d(m_normal.dot(y_comp), -m_normal.dot(y_comp));
    mz = Eigen::RowVector2d(m_normal.dot(z_comp), -m_normal.dot(z_comp));

    return;
}

void ContactGraph::get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
                                      Eigen::RowVector4d &mx, Eigen::RowVector4d &my, Eigen::RowVector4d &mz)
{

    Eigen::Vector3d x_comp(1, 0 ,0);
    Eigen::Vector3d y_comp(0, 1 ,0);
    Eigen::Vector3d z_comp(0, 0 ,1);
    Eigen::Vector3d m_normal = -r.cross(n);
    Eigen::Vector3d m_u_fric = -r.cross(u);
    Eigen::Vector3d m_v_fric = -r.cross(v);
    mx = Eigen::RowVector4d(m_normal.dot(x_comp), -m_normal.dot(x_comp), m_u_fric.dot(x_comp), m_v_fric.dot(x_comp));
    my = Eigen::RowVector4d(m_normal.dot(y_comp), -m_normal.dot(y_comp), m_u_fric.dot(y_comp), m_v_fric.dot(y_comp));
    mz = Eigen::RowVector4d(m_normal.dot(z_comp), -m_normal.dot(z_comp), m_u_fric.dot(z_comp), m_v_fric.dot(z_comp));
}

void ContactGraph::get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction)
{
    if(!withFriction){
        /*
     * 1.0 get interface and part
     */
        shared_ptr<ContactGraphEdge> edge = edges[edgeID];
        shared_ptr<ContactGraphNode> part = nodes[partID];

        /*
         * 2.0 initialize the matrix Ajk
         */

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 2 * num_vks);

        int stack_vk = 0;
        for(int id = 0; id < edge->polygons.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            int num_vk = poly.points.size();
            /*
             * 3.0 filling the force fx, fy, fz, term
             */
            Eigen::Vector3d normal, u_fric, v_fric;
            Eigen::RowVector2d fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 2 * stack_vk, 1, 2 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 2 * stack_vk, 1, 2 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 2 * stack_vk, 1, 2 * num_vk) = fkz.replicate(1, num_vk);

            /*
             * 4.0 filling the torque mx, my, mz term
             */
            for(int jd = 0; jd < num_vk; jd++)
            {
                Eigen::Vector3d r = (poly.points[jd] - part->centroid).cast<double>();
                Eigen::RowVector2d mx, my, mz;
                get_moment_from_norm_fric_vertex(normal, u_fric, v_fric, r, mx, my, mz);
                Ajk.block(3, jd * 2 + 2 * stack_vk, 1, 2) = mx;
                Ajk.block(4, jd * 2 + 2 * stack_vk, 1, 2) = my;
                Ajk.block(5, jd * 2 + 2 * stack_vk, 1, 2) = mz;
            }
            stack_vk += num_vk;
        }
    }
    else{
        /*
        * 1.0 get interface and part
        */
        shared_ptr<ContactGraphEdge> edge = edges[edgeID];
        shared_ptr<ContactGraphNode> part = nodes[partID];

        /*
         * 2.0 initialize the matrix Ajk
         */

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 4 * num_vks);
        int stack_vk = 0;
        for(int id = 0; id < edge->polygons.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            int num_vk = poly.points.size();
            /*
             * 3.0 filling the force fx, fy, fz, term
             */
            Eigen::Vector3d normal, u_fric, v_fric;
            Eigen::RowVector4d fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 4 * stack_vk, 1, 4 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 4 * stack_vk, 1, 4 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 4 * stack_vk, 1, 4 * num_vk) = fkz.replicate(1, num_vk);

            /*
             * 4.0 filling the torque mx, my, mz term
             */
            for(int jd = 0; jd < num_vk; jd++)
            {
                Eigen::Vector3d r = (poly.points[jd] - part->centroid).cast<double>();
                Eigen::RowVector4d mx, my, mz;
                get_moment_from_norm_fric_vertex(normal, u_fric, v_fric, r, mx, my, mz);
                Ajk.block(3, jd * 4 + 4 * stack_vk, 1, 4) = mx;
                Ajk.block(4, jd * 4 + 4 * stack_vk, 1, 4) = my;
                Ajk.block(5, jd * 4 + 4 * stack_vk, 1, 4) = mz;
            }
            stack_vk += num_vk;
        }
    }


    return;
}

void ContactGraph::computeEquilibriumMatrix(Eigen::MatrixXd &Aeq,  bool withFriction) {
    if (!withFriction)
    {
        /*
         * 1.0 init the matrix
         */
        vector<int> row_start_index;
        int start_index = 0;
        for (shared_ptr<ContactGraphEdge> edge:edges) {
            row_start_index.push_back(start_index);
            //std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 2;
        }
        Aeq = Eigen::MatrixXd::Zero(dynamic_nodes.size() * 6, start_index);

        /*
         * 2.0 build matrix
         */

        for (int id = 0; id < edges.size(); id++) {
            shared_ptr<ContactGraphEdge> edge = edges[id];
            shared_ptr<ContactGraphNode> partA = nodes[edge->partIDA];
            shared_ptr<ContactGraphNode> partB = nodes[edge->partIDB];
            int dyn_partIDA = partA->dynamicID;
            int dyn_partIDB = partB->dynamicID;
            int num_vk = edge->num_points();
            if (!partA->isBoundary) {
                Eigen::MatrixXd Ajk;
                get_A_j_k(partA->staticID, id, Ajk);
                Aeq.block(6 * dyn_partIDA, row_start_index[id], 6, 2 * num_vk) = Ajk;
            }
            if (!partB->isBoundary) {
                Eigen::MatrixXd Ajk;
                get_A_j_k(partB->staticID, id, Ajk);
                Aeq.block(6 * dyn_partIDB, row_start_index[id], 6, 2 * num_vk) = Ajk;
            }
        }
    }

    else{
        /*
         * 1.0 init the matrix
         */
        vector<int> row_start_index;
        int start_index = 0;
        for (shared_ptr<ContactGraphEdge> edge:edges) {
            row_start_index.push_back(start_index);
            //std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 4;
        }
        Aeq = Eigen::MatrixXd::Zero(dynamic_nodes.size() * 6, start_index);

        /*
         * 2.0 build matrix
         */

        for (int id = 0; id < edges.size(); id++) {
            shared_ptr<ContactGraphEdge> edge = edges[id];
            shared_ptr<ContactGraphNode> partA = nodes[edge->partIDA];
            shared_ptr<ContactGraphNode> partB = nodes[edge->partIDB];
            int dyn_partIDA = partA->dynamicID;
            int dyn_partIDB = partB->dynamicID;
            int num_vk = edge->num_points();
            if (!partA->isBoundary) {
                Eigen::MatrixXd Ajk;
                get_A_j_k(partA->staticID, id, Ajk, withFriction);
                Aeq.block(6 * dyn_partIDA, row_start_index[id], 6, 4 * num_vk) = Ajk;
            }
            if (!partB->isBoundary) {
                Eigen::MatrixXd Ajk;
                get_A_j_k(partB->staticID, id, Ajk, withFriction);
                Aeq.block(6 * dyn_partIDB, row_start_index[id], 6, 4 * num_vk) = Ajk;
            }
        }
    }

}


double ContactGraph::computeEquilibriumMatrixConditonalNumber()
{
    Eigen::MatrixXd Aeq;
    computeEquilibriumMatrix(Aeq);

    if(!Aeq.isZero()){
        Eigen::MatrixXd TAeq(Aeq.rows(), Aeq.cols() / 2);
        for(int id = 0; id < Aeq.cols() / 2; id++)
        {
            TAeq.col(id) = Aeq.col(id * 2);
        }
        Eigen::MatrixXd TransTAeq = TAeq.transpose();
        Eigen::MatrixXd TAeqinv = TAeq.completeOrthogonalDecomposition().pseudoInverse();
        return TAeqinv.norm() * TAeq.norm();
    }
    else{
        return 0;
    }

}

/*************************************************
*
*               Assembling Sequence
*
*************************************************/

void ContactGraph::computeAssemblingSequence(ContactGraphAssemblySequence &sequence, const vector<int> &nonBoundaryParts, int numLayers)
{
    clusterAssemblingLayers(sequence, numLayers);
    map<ContactGraphNode *, bool> removeParts;
    map<ContactGraphNode *, bool> nonBoundary;
    for(int partID : nonBoundaryParts)
    {
        nonBoundary[nodes[partID].get()] = true;
        sequence.layerParts.back().push_back(nodes[partID]);
        sequence.partLayers[nodes[partID].get()] = numLayers - 1;
    }

    for(int layer = numLayers - 1; layer >= 0; layer --)
    {
        vector<weak_ptr<ContactGraphNode>> layerNodes = sequence.layerParts[layer];

        while(!layerNodes.empty())
        {
            vector<pair<weak_ptr<ContactGraphNode>, float>> mobility;

            //compute Part mobility
            for(weak_ptr<ContactGraphNode> node: layerNodes)
            {
                pair<weak_ptr<ContactGraphNode>, float> node_mobility;
                node_mobility.first = node;
                PolyhedralCone cone;
                node_mobility.second = computePartMobility(node.lock(), removeParts, cone);
                mobility.push_back(node_mobility);
            }

            shared_ptr<ContactGraphNode> node;
            if(std::all_of(mobility.begin(), mobility.end(),[](pair<weak_ptr<ContactGraphNode>, float> x){return x.second < 0;}))
            {
                //no part can be disassembled
                sort(mobility.begin(), mobility.end(), [](pair<weak_ptr<ContactGraphNode>, float> a, pair<weak_ptr<ContactGraphNode>, float> b){
                    return a.first.lock()->height() > b.first.lock()->height();
                });

                node = mobility.front().first.lock();
                sequence.orderParts.push_back(node);
                sequence.splitParts[node.get()] = true;
                PolyhedralCone cone;
                sequence.orderCones.push_back(cone);
                removeParts[node.get()] = true;
            }
            else {
                //disassembled the highest mobility part
                sort(mobility.begin(), mobility.end(), [](pair<weak_ptr<ContactGraphNode>, float> a, pair<weak_ptr<ContactGraphNode>, float> b){
                    return a.second > b.second;
                });

                node = mobility.front().first.lock();
                PolyhedralCone cone;
                computePartMobility(node, removeParts, cone);
                sequence.orderCones.push_back(cone);
                removeParts[node.get()] = true;
                sequence.orderParts.push_back(node);
            }

            for(auto it = layerNodes.begin(); it != layerNodes.end();){
                if((*it).lock() == node){
                    it = layerNodes.erase(it);
                }
                else{
                    it++;
                }
            }

        }
    }

    for(int id = 0; id < nodes.size(); id++){
        if(nodes[id]->isBoundary && nonBoundary[nodes[id].get()] != true){
            sequence.boundaryParts[nodes[id].get()] = true;
        }
    }
}

double ContactGraph::computePartMobility(shared_ptr<ContactGraphNode> part, map<ContactGraphNode *, bool> &removeParts, PolyhedralCone &cone)
{
    stdvec_Vector3d normals;
    for(int id = 0; id < part->neighbors.size(); id++)
    {
        shared_ptr<ContactGraphNode> neighbor = part->neighbors[id].first.lock();
        if(neighbor == nullptr) continue;
        shared_ptr<ContactGraphEdge> edge = part->neighbors[id].second.lock();
        if(removeParts[neighbor.get()] == false)
        {
            for(int poly_id = 0; poly_id < edge->polygons.size(); poly_id++)
            {
                EigenPoint normal = edge->getContactNormal(neighbor->staticID, poly_id);
                normals.push_back(Eigen::Vector3d(normal[0], normal[1], normal[2]));
            }
        }
    }

    Matrix3d N(3, normals.size());
    for(int id = 0; id < normals.size(); id++){
        N.col(id) = normals[id];
    }

    cone = PolyhedralCone(N);
    return cone.solidAngle();
}

void ContactGraph::clusterAssemblingLayers(ContactGraphAssemblySequence &sequence, int numLayers)
{
    //kmeans, y is the height

    vector<float> heights;
    for(int id = 0; id < dynamic_nodes.size(); id++){
        float height = dynamic_nodes[id].lock()->height();
        heights.push_back(height);
    }

    sort(heights.begin(), heights.end());
    int num_heights = heights.size();
    vector<float> cluster;

    for(int id = 0; id < numLayers; id++)
    {
        cluster.push_back(heights[(float)(num_heights) / numLayers * id]);
    }

    double times = getVarList()->get<int>("kmeans_iteration");
    while(times--)
    {
        sequence.partLayers.clear();
        sequence.layerParts.clear();
        sequence.layerParts.resize(numLayers);
        for(int id = 0; id < dynamic_nodes.size(); id++){
            shared_ptr<ContactGraphNode> node = dynamic_nodes[id].lock();
            float min_dist = std::fabs(cluster[0] - node->height());
            int layer = 0;
            for(int jd = 1; jd < cluster.size(); jd++){
                float dist = std::fabs(cluster[jd] - node->height());
                if(dist < min_dist){
                    layer = jd;
                    min_dist = dist;
                }
            }

            //confirm the node belong to this layer
            sequence.layerParts[layer].push_back(node);
            sequence.partLayers[node.get()] = layer;
        }

        //update cluster
        cluster.clear();
        for(int id = 0; id < numLayers; id++)
        {
            float height = 0;
            for(int jd = 0; jd < sequence.layerParts[id].size(); jd++){
                shared_ptr<ContactGraphNode> node = sequence.layerParts[id][jd].lock();
                height += node->height();
            }
            height /= sequence.layerParts[id].size();
            cluster.push_back(height);
        }
    }

    return;
}