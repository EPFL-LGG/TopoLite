//
// Created by ziqwang on 2019-12-10.
//

#include "InterlockingSolver.h"

/*************************************************
*
*      Building block for the Interlocking Matrix
*
*************************************************/

void InterlockingSolver::get_force_from_norm_fric(Eigen::Vector3d n,       Eigen::Vector3d u,       Eigen::Vector3d v,
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

void InterlockingSolver::get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                            Eigen::RowVector4d    &fkx, Eigen::RowVector4d &fky, Eigen::RowVector4d &fkz){
    Eigen::Vector3d x_comp(1, 0 ,0);
    Eigen::Vector3d y_comp(0, 1 ,0);
    Eigen::Vector3d z_comp(0, 0 ,1);
    fkx = Eigen::RowVector4d(n.dot(x_comp), -n.dot(x_comp), u.dot(x_comp), v.dot(x_comp));
    fky = Eigen::RowVector4d(n.dot(y_comp), -n.dot(y_comp), u.dot(y_comp), v.dot(y_comp));
    fkz = Eigen::RowVector4d(n.dot(z_comp), -n.dot(z_comp), u.dot(z_comp), v.dot(z_comp));
}

void InterlockingSolver::get_moment_from_norm_fric_vertex(Eigen::Vector3d n, Eigen::Vector3d u, Eigen::Vector3d v,
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

void InterlockingSolver::get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
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

void InterlockingSolver::get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction)
{
    if(!withFriction){

        // 1.0 get interface and part

        shared_ptr<ContactGraphEdge> edge = graph->edges[edgeID];
        shared_ptr<ContactGraphNode> part = graph->nodes[partID];

        // 2.0 initialize the matrix Ajk

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 2 * num_vks);

        int stack_vk = 0;
        for(size_t id = 0; id < edge->polygons.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            int num_vk = poly.points.size();

            // 3.0 filling the force fx, fy, fz, term

            Eigen::Vector3d normal, u_fric, v_fric;
            Eigen::RowVector2d fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 2 * stack_vk, 1, 2 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 2 * stack_vk, 1, 2 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 2 * stack_vk, 1, 2 * num_vk) = fkz.replicate(1, num_vk);

            // 4.0 filling the torque mx, my, mz term
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
    else {

        // 1.0 get interface and part

        shared_ptr<ContactGraphEdge> edge = graph->edges[edgeID];
        shared_ptr<ContactGraphNode> part = graph->nodes[partID];

        // 2.0 initialize the matrix Ajk

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 4 * num_vks);
        int stack_vk = 0;
        for(size_t id = 0; id < edge->polygons.size(); id++)
        {
            ContactPolygon poly = edge->polygons[id];
            int num_vk = poly.points.size();

            // 3.0 filling the force fx, fy, fz, term

            Eigen::Vector3d normal, u_fric, v_fric;
            Eigen::RowVector4d fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 4 * stack_vk, 1, 4 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 4 * stack_vk, 1, 4 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 4 * stack_vk, 1, 4 * num_vk) = fkz.replicate(1, num_vk);


            // 4.0 filling the torque mx, my, mz term

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


/*************************************************
*
*           Compute Interlocking Matrix
*
*************************************************/

void InterlockingSolver::computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: graph->edges) {

        int iA = graph->nodes[edge->partIDA]->dynamicID;

        int iB = graph->nodes[edge->partIDB]->dynamicID;

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
    size = Eigen::Vector2i(rowID, 3 * graph->dynamic_nodes.size());
}

void InterlockingSolver::computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge> edge: graph->edges)
    {
        shared_ptr<ContactGraphNode> nodeA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode> nodeB = graph->nodes[edge->partIDB];

        int iA = nodeA->dynamicID;
        int iB = nodeB->dynamicID;

        EigenPoint ctA = nodeA->centroid;
        EigenPoint ctB = nodeB->centroid;

        for(size_t id = 0; id < edge->normals.size(); id++)
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
    size = Eigen::Vector2i(rowID, 6 * graph->dynamic_nodes.size());
}

void InterlockingSolver::computeTranslationalInterlockingMatrixDense(Eigen::MatrixXd &mat){
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeTranslationalInterlockingMatrix(tris, size);

    mat = Eigen::MatrixXd::Zero(size[0], size[1]);

    for(EigenTriple tri: tris){
        mat(tri.row(), tri.col()) = tri.value();
    }

    return;
}

void InterlockingSolver::computeRotationalInterlockingMatrixDense(Eigen::MatrixXd &mat)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeRotationalInterlockingMatrix(tris, size);

    mat = Eigen::MatrixXd::Zero(size[0], size[1]);

    for(EigenTriple tri: tris){
        mat(tri.row(), tri.col()) = tri.value();
    }

    return;
}


void InterlockingSolver::computeRotationalInterlockingMatrixSparse(EigenSpMat &spatMat)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeRotationalInterlockingMatrix(tris, size);
    spatMat = EigenSpMat(size[0], size[1]);
    spatMat.setFromTriplets(tris.begin(), tris.end());
}



void InterlockingSolver::computeEquilibriumMatrix(Eigen::MatrixXd &Aeq,  bool withFriction) {
    if (!withFriction)
    {

        // 1.0 init the matrix

        vector<int> row_start_index;
        int start_index = 0;
        for (shared_ptr<ContactGraphEdge> edge: graph->edges) {
            row_start_index.push_back(start_index);
            // std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 2;
        }
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);


        // 2.0 build matrix


        for (size_t id = 0; id < graph->edges.size(); id++) {
            shared_ptr<ContactGraphEdge> edge = graph->edges[id];
            shared_ptr<ContactGraphNode> partA = graph->nodes[edge->partIDA];
            shared_ptr<ContactGraphNode> partB = graph->nodes[edge->partIDB];
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

        // 1.0 init the matrix

        vector<int> row_start_index;
        int start_index = 0;
        for (shared_ptr<ContactGraphEdge> edge: graph->edges) {
            row_start_index.push_back(start_index);
            // std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 4;
        }
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);

        // 2.0 build matrix

        for (size_t id = 0; id < graph->edges.size(); id++) {
            shared_ptr<ContactGraphEdge> edge = graph->edges[id];
            shared_ptr<ContactGraphNode> partA = graph->nodes[edge->partIDA];
            shared_ptr<ContactGraphNode> partB = graph->nodes[edge->partIDB];
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

double InterlockingSolver::computeEquilibriumMatrixConditonalNumber()
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

