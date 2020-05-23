//
// Created by ziqwang on 2020-02-14.
//
#include "ListVector3.h"
bool compareListVector3(const std::vector<Eigen::Vector3d> &A, const std::vector<Eigen::Vector3d> &B)
{

    if(A.size() != B.size()) return false;

    for(int id = 0; id < A.size(); id++)
    {
        int jd = 0;
        for(; jd < B.size(); jd++){
            if(Approx(0) == (A[id] - B[jd]).norm()){
                break;
            }
        }

        if(jd == B.size())
            return false;
    }
    return true;

}

void convertString3ToListVector3(std::string data, std::vector<Eigen::Vector3d> &pts)
{

    std::stringstream iss(data);
    double number;
    int index = 0;
    while(iss >> number){

        if(index % 3 == 0){
            pts.push_back(Eigen::Vector3d(0, 0, 0));
        }

        pts.back()[index % 3] = number;
        index += 1;
    }

    return;
}

void convertString2ToListVector3(std::string data, std::vector<Eigen::Vector3d> &pts)
{

    std::stringstream iss(data);
    double number;
    int index = 0;
    while(iss >> number){

        if(index % 2 == 0){
            pts.push_back(Eigen::Vector3d(0, 0, 0));
        }

        pts.back()[index % 2] = number;
        index += 1;
    }

    return;
}

void printListVector3(std::vector<Eigen::Vector3d> &list){
    for(Eigen::Vector3d pt : list){
        std::cout << pt.transpose() << std::endl;
    }
}