//
// Created by ziqwang on 31.03.19.
//

#ifndef TOPOLOCKCREATOR_CONVEXHULL3D_H
#define TOPOLOCKCREATOR_CONVEXHULL3D_H
void computeQuickHull(vector<Vector3f> &pointList, vector<Vector3f> &ver, vector<Vector3i> &tri);
void computeQuickHullMatrix(vector<Vector3f> &pointList, Eigen::MatrixXd &V, Eigen::MatrixXi &F);
#endif //TOPOLOCKCREATOR_CONVEXHULL3D_H
