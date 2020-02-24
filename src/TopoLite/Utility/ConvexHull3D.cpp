//
// Created by ziqwang on 31.03.19.
//
#include "QuickHull.hpp"
using Eigen::Vector3i;

template<typename Scalar>
void ConvexHull3D<Scalar>::compute(ListVector3 &pointList, ListVector3 &ver, ListVector3i &tri)
{
    quickhull::QuickHull<Scalar> qh;
    std::vector<quickhull::Vector3<Scalar>> pointCloud;
    for(size_t id = 0; id < pointList.size(); id++)
    {
        pointCloud.push_back(quickhull::Vector3<Scalar>(pointList[id].x(), pointList[id].y(), pointList[id].z()));
    }

    auto hull = qh.getConvexHull(pointCloud, true, false, 1e-4);
    auto index_buff = hull.getIndexBuffer();
    auto vertex_buff = hull.getVertexBuffer();

    ver.clear();
    tri.clear();

    for(int id = 0; id < index_buff.size() / 3; id++){
        int a = index_buff[id * 3];
        int b = index_buff[id * 3 + 1];
        int c = index_buff[id * 3 + 2];
        tri.push_back(Vector3i(a, c, b));
    }

    for(size_t id = 0; id < vertex_buff.size(); id++){
        Scalar a = vertex_buff[id].x;
        Scalar b = vertex_buff[id].y;
        Scalar c = vertex_buff[id].z;
        ver.push_back(Vector3(a, b, c));
    }

    return;
}

template<typename Scalar>
void ConvexHull3D<Scalar>::compute(ListVector3 &pointList, Matrix<Scalar, Dynamic, Dynamic> &V, Matrix<int, Dynamic, Dynamic> &F)
{
    ListVector3 ver;
    ListVector3i tri;
    computeQuickHull(pointList, ver, tri);
    V = Eigen::MatrixXd(ver.size(), 3);
    F = Eigen::MatrixXi(tri.size(), 3);

    for(size_t id = 0; id < ver.size(); id++){
        V.row(id) = Eigen::RowVector3d(ver[id].x(), ver[id].y(), ver[id].z());
    }
    for(size_t id = 0; id < tri.size(); id++){
        F.row(id) = Eigen::RowVector3i(tri[id].x(), tri[id].y(), tri[id].z());
    }
    return;
}
