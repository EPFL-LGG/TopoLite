//
// Created by ziqwang on 31.03.19.
//
#include "QuickHull.hpp"
#include "vec.h"
#include <Eigen/Dense>

void computeQuickHull(std::vector<Vector3f> &pointList, std::vector<Vector3f> &ver, std::vector<Vector3i> &tri)
{
    quickhull::QuickHull<float> qh;
    std::vector<quickhull::Vector3<float>> pointCloud;
    for(int id = 0; id < pointList.size(); id++){
        pointCloud.push_back(quickhull::Vector3<float>(pointList[id].x, pointList[id].y, pointList[id].z));
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

    for(int id = 0; id < vertex_buff.size(); id++){
        float a = vertex_buff[id].x;
        float b = vertex_buff[id].y;
        float c = vertex_buff[id].z;
        ver.push_back(Vector3f(a, b, c));
    }
}

void computeQuickHullMatrix(std::vector<Vector3f> &pointList, Eigen::MatrixXd &V, Eigen::MatrixXi &F)
{
    std::vector<Vector3f> ver;
    std::vector<Vector3i> tri;
    computeQuickHull(pointList, ver, tri);
    V = Eigen::MatrixXd(ver.size(), 3);
    F = Eigen::MatrixXi(tri.size(), 3);

    for(int id = 0; id < ver.size(); id++){
        V.row(id) = Eigen::RowVector3d(ver[id].x, ver[id].y, ver[id].z);
    }
    for(int id = 0; id < tri.size(); id++){
        F.row(id) = Eigen::RowVector3i(tri[id].x, tri[id].y, tri[id].z);
    }
}
