//
// Created by ziqwang on 2020-02-14.
//

#ifndef TOPOLITE_LISTVECTOR3_H
#define TOPOLITE_LISTVECTOR3_H

#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
bool compareListVector3(const std::vector<Eigen::Vector3d> &A, const std::vector<Eigen::Vector3d> &B);
void convertString3ToListVector3(std::string data, std::vector<Eigen::Vector3d> &pts);
void convertString2ToListVector3(std::string data, std::vector<Eigen::Vector3d> &pts);
void printListVector3(std::vector<Eigen::Vector3d> &list);

#endif //TOPOLITE_LISTVECTOR3_H
