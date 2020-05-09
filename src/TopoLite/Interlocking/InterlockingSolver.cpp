//
// Created by ziqwang on 2019-12-10.
//

#include "InterlockingSolver.h"

/*************************************************
*
*      Building block for the Interlocking Matrix
*
*************************************************/

template<typename Scalar>
void InterlockingSolver<Scalar>::get_force_from_norm_fric(Vector3 n,       Vector3 u,       Vector3 v,
                                            RowVector2 &fkx, RowVector2 &fky, RowVector2 &fkz)
{
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    fkx = RowVector2(n.dot(x_comp), -n.dot(x_comp));
    fky = RowVector2(n.dot(y_comp), -n.dot(y_comp));
    fkz = RowVector2(n.dot(z_comp), -n.dot(z_comp));
    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::get_force_from_norm_fric(Vector3    n,    Vector3   u,  Vector3   v,
                                            RowVector4    &fkx, RowVector4 &fky, RowVector4 &fkz){
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    fkx = RowVector4(n.dot(x_comp), -n.dot(x_comp), u.dot(x_comp), v.dot(x_comp));
    fky = RowVector4(n.dot(y_comp), -n.dot(y_comp), u.dot(y_comp), v.dot(y_comp));
    fkz = RowVector4(n.dot(z_comp), -n.dot(z_comp), u.dot(z_comp), v.dot(z_comp));
}

template<typename Scalar>
void InterlockingSolver<Scalar>::get_moment_from_norm_fric_vertex(Vector3 n, Vector3 u, Vector3 v,
                                                    Vector3 r, RowVector2 &mx, RowVector2 &my,
                                                    RowVector2 &mz)
{
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
//    Vector3 m_normal = n.cross(r);
//    Vector3 m_u_fric = u.cross(r);
//    Vector3 m_v_fric = v.cross(r);
    Vector3 m_normal = -r.cross(n);
    Vector3 m_u_fric = -r.cross(u);
    Vector3 m_v_fric = -r.cross(v);

    mx = RowVector2(m_normal.dot(x_comp), -m_normal.dot(x_comp));
    my = RowVector2(m_normal.dot(y_comp), -m_normal.dot(y_comp));
    mz = RowVector2(m_normal.dot(z_comp), -m_normal.dot(z_comp));

    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::get_moment_from_norm_fric_vertex(Vector3 n,   Vector3 u,   Vector3 v, Vector3 r,
                                                    RowVector4 &mx, RowVector4 &my, RowVector4 &mz)
{

    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    Vector3 m_normal = -r.cross(n);
    Vector3 m_u_fric = -r.cross(u);
    Vector3 m_v_fric = -r.cross(v);
    mx = RowVector4(m_normal.dot(x_comp), -m_normal.dot(x_comp), m_u_fric.dot(x_comp), m_v_fric.dot(x_comp));
    my = RowVector4(m_normal.dot(y_comp), -m_normal.dot(y_comp), m_u_fric.dot(y_comp), m_v_fric.dot(y_comp));
    mz = RowVector4(m_normal.dot(z_comp), -m_normal.dot(z_comp), m_u_fric.dot(z_comp), m_v_fric.dot(z_comp));
}

template<typename Scalar>
void InterlockingSolver<Scalar>::get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction)
{
    if(!withFriction){

        // 1.0 get interface and part

        shared_ptr<ContactGraphEdge<Scalar>> edge = graph->edges[edgeID];
        shared_ptr<ContactGraphNode<Scalar>> part = graph->nodes[partID];

        // 2.0 initialize the matrix Ajk

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 2 * num_vks);

        int stack_vk = 0;
        for(size_t id = 0; id < edge->polygons.size(); id++)
        {
            shared_ptr<_Polygon<Scalar>> poly = edge->polygons[id];
            int num_vk = poly.points.size();

            // 3.0 filling the force fx, fy, fz, term

            Vector3 normal, u_fric, v_fric;
            RowVector2 fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 2 * stack_vk, 1, 2 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 2 * stack_vk, 1, 2 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 2 * stack_vk, 1, 2 * num_vk) = fkz.replicate(1, num_vk);

            // 4.0 filling the torque mx, my, mz term
            for(int jd = 0; jd < num_vk; jd++)
            {
                Vector3 r = poly.points[jd] - part->centroid;
                RowVector2 mx, my, mz;
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

        shared_ptr<ContactGraphEdge<Scalar>> edge = graph->edges[edgeID];
        shared_ptr<ContactGraphNode<Scalar>> part = graph->nodes[partID];

        // 2.0 initialize the matrix Ajk

        int num_vks = edge->num_points();
        Ajk = Eigen::MatrixXd(6, 4 * num_vks);
        int stack_vk = 0;
        for(size_t id = 0; id < edge->polygons.size(); id++)
        {
            shared_ptr<_Polygon<Scalar>> poly = edge->polygons[id];
            int num_vk = poly.points.size();

            // 3.0 filling the force fx, fy, fz, term

            Vector3 normal, u_fric, v_fric;
            RowVector4 fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
            Ajk.block(0, 4 * stack_vk, 1, 4 * num_vk) = fkx.replicate(1, num_vk);
            Ajk.block(1, 4 * stack_vk, 1, 4 * num_vk) = fky.replicate(1, num_vk);
            Ajk.block(2, 4 * stack_vk, 1, 4 * num_vk) = fkz.replicate(1, num_vk);


            // 4.0 filling the torque mx, my, mz term

            for(int jd = 0; jd < num_vk; jd++)
            {
                Vector3 r = poly.points[jd] - part->centroid;
                RowVector4 mx, my, mz;
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

template<typename Scalar>
void InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge<Scalar>> edge: graph->edges) {

        int iA = graph->nodes[edge->partIDA]->dynamicID;

        int iB = graph->nodes[edge->partIDB]->dynamicID;

        Vector3 nrm = edge->normal;
        for (size_t id = 0; id < edge->size(); id++) {
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

template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int rowID = 0;
    tri.clear();
    for(shared_ptr<ContactGraphEdge<Scalar>> edge: graph->edges)
    {
        shared_ptr<ContactGraphNode<Scalar>> nodeA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<Scalar>> nodeB = graph->nodes[edge->partIDB];

        int iA = nodeA->dynamicID;
        int iB = nodeB->dynamicID;

        Vector3 ctA = nodeA->centroid;
        Vector3 ctB = nodeB->centroid;

        Vector3 nrm = edge->normal;
        for(size_t id = 0; id < edge->size(); id++)
        {
            shared_ptr<_Polygon<Scalar>> poly = edge->polygons[id];
            for(pVertex ver: poly->vers)
            {
                Vector3 pt = ver->pos;
                if (iA != -1)
                {
                    Vector3 mt = (pt - ctA).cross(nrm);
                    tri.push_back(EigenTriple(rowID, 6 * iA, -nrm[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 1, -nrm[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 2, -nrm[2]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 3, -mt[0]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 4, -mt[1]));
                    tri.push_back(EigenTriple(rowID, 6 * iA + 5, -mt[2]));
                }

                if (iB != -1) {
                    Vector3 mt = (pt - ctB).cross(nrm);
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

template<typename Scalar>
void InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrixDense(Eigen::MatrixXd &mat){
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeTranslationalInterlockingMatrix(tris, size);

    mat = Eigen::MatrixXd::Zero(size[0], size[1]);

    for(EigenTriple tri: tris){
        mat(tri.row(), tri.col()) = tri.value();
    }

    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrixDense(Eigen::MatrixXd &mat)
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


template<typename Scalar>
void InterlockingSolver<Scalar>::computeRotationalInterlockingMatrixSparse(EigenSpMat &spatMat)
{
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    computeRotationalInterlockingMatrix(tris, size);
    spatMat = EigenSpMat(size[0], size[1]);
    spatMat.setFromTriplets(tris.begin(), tris.end());
}


template<typename Scalar>
void InterlockingSolver<Scalar>::appendAuxiliaryVariables(vector<EigenTriple> &tri, Eigen::Vector2i &size)
{
    int num_row = size[0];
    int num_col = size[1];

    for(size_t id = 0; id < num_row; id++){
        tri.push_back(EigenTriple(id, num_col + id, -1));
    }
    size[1] += num_row;
    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::appendMergeConstraints(vector<EigenTriple> &tri, Eigen::Vector2i &size, bool isRotation)
{
    int dimension = (isRotation ? 6 : 3);
    for(int id = 0;id < graph->merged_nodes.size(); id++)
    {
        wpContactGraphNode A = graph->merged_nodes[id].first;
        wpContactGraphNode B = graph->merged_nodes[id].second;
        int iA = A.lock()->dynamicID;
        int iB = B.lock()->dynamicID;
        if(iA != -1 && iB != -1)
        {
            for(int index = 0; index < dimension; index++)
            {
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iA + index, 1));
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iB + index, -1));

                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iA + index, -1));
                tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iB + index, 1));
            }
        }
        else{
            for(int index = 0; index < dimension; index++){
                if(iA != -1) {
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index, dimension * iA + index, 1));
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iA + index, -1));
                }
                if(iB != -1){
                    tri.push_back(EigenTriple(size[0] + id * dimension + index, dimension * iB + index, 1));
                    tri.push_back(EigenTriple(size[0] + id * dimension * 2 + index + 1, dimension * iB + index, -1));

                }
            }
        }
    }
    size[0] += graph->merged_nodes.size() * dimension * 2;
    return;
}

template<typename Scalar>
void InterlockingSolver<Scalar>::computeEquilibriumMatrix(Eigen::MatrixXd &Aeq,  bool withFriction) {
    if (!withFriction)
    {

        // 1.0 init the matrix

        vector<int> row_start_index;
        int start_index = 0;
        for (shared_ptr<ContactGraphEdge<Scalar>> edge: graph->edges) {
            row_start_index.push_back(start_index);
            // std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 2;
        }
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);


        // 2.0 build matrix


        for (size_t id = 0; id < graph->edges.size(); id++) {
            shared_ptr<ContactGraphEdge<Scalar>> edge = graph->edges[id];
            shared_ptr<ContactGraphNode<Scalar>> partA = graph->nodes[edge->partIDA];
            shared_ptr<ContactGraphNode<Scalar>> partB = graph->nodes[edge->partIDB];
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
        for (shared_ptr<ContactGraphEdge<Scalar>> edge: graph->edges) {
            row_start_index.push_back(start_index);
            // std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
            start_index += edge->num_points() * 4;
        }
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);

        // 2.0 build matrix

        for (size_t id = 0; id < graph->edges.size(); id++) {
            shared_ptr<ContactGraphEdge<Scalar>> edge = graph->edges[id];
            shared_ptr<ContactGraphNode<Scalar>> partA = graph->nodes[edge->partIDA];
            shared_ptr<ContactGraphNode<Scalar>> partB = graph->nodes[edge->partIDB];
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

template <typename Scalar>
Scalar InterlockingSolver<Scalar>::computeEquilibriumMatrixConditonalNumber()
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


// No need to call this TemporaryFunction() function,
// it's just to avoid link error.
void TemporaryFunction_InterlockingSolver ()
{
    InterlockingSolver<double> solver(nullptr, nullptr);

    vector<InterlockingSolver<double>::EigenTriple> tri;
    Eigen::Vector2i size;
    bool isRotation;
    solver.appendMergeConstraints(tri, size, isRotation);
    solver.appendAuxiliaryVariables(tri, size);
    solver.computeRotationalInterlockingMatrix(tri, size);
    solver.computeTranslationalInterlockingMatrix(tri, size);
}
