//
// Created by ziqwang on 28.03.19.
//

#ifndef TOPOLOCKCREATOR_POLYHEDRALCONE_H
#define TOPOLOCKCREATOR_POLYHEDRALCONE_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <map>

#include "Utility/HelpDefine.h"
using std::map;
using stdvec_Vector3d = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>;
using Matrix3d = Eigen::Matrix<double, 3, Eigen::Dynamic>;

class PolyhedralCone
{
public:
    Matrix3d E;
    Matrix3d m_N;
    double solid_angle;

public:

    PolyhedralCone()
    {
        m_N = Matrix3d::Zero(3, 0);
    }

    PolyhedralCone(Matrix3d _normals)
    {
        m_N = _normals;
        removeDuplicate(m_N);
    }

public:
    /***
     * @brief: compute the solid angle of the pymarid by using the formula in: https://arxiv.org/pdf/1205.1396.pdf
     *         you can get the pymarid region by using the matrix E after calling this function.
     * @return: -1      means the intersection region is empty
     *          =0      means the intersection region is not empty but the area is 0
     *          >0      means the intersection region is not empty and the solid angle exists.
     *          =2PI    means half sphere
     *          =4PI    means whole sphere
     */
    float solidAngle()
    {
        E.setZero();
        if(m_N.cols() == 0)
        {
            solid_angle = 4 * M_PI;
            return 4 * M_PI;
        }

        if(m_N.cols() == 1)
        {
            solid_angle = 2 * M_PI;
            return 2 * M_PI;
        }

        computeEdge(E);
        if(E.cols() < 1)
        {
            solid_angle = -1;
            return -1;
        }
        else if(E.cols() < 2)
        {
            solid_angle = 0;
            return 0;
        }
        else if(E.cols() == 2)
        {
            double min_a = std::numeric_limits<double>::max();
            for(int id = 0; id < m_N.cols(); id++)
            {
                for(int jd = id + 1; jd < m_N.cols(); jd++)
                {
                    double a = 2 * M_PI - 2 * angle(m_N.col(id), m_N.col(jd));
                    if(min_a > a) min_a = a;
                }
            }
            solid_angle = min_a;
            return min_a;
        }
        else
        {
            orderEdge(E);
            int N = E.cols();
            std::complex<double> theta(1, 0);
            double angle = 2 * M_PI;
            for(int id = 0; id < N; id++)
            {
                Eigen::Vector3d e_m, e_p, e_n;
                e_m = E.col((id - 1 + N) % N);
                e_p = E.col(id);
                e_n = E.col((id + 1) % N);
                double a = e_m.dot(e_n);
                double b = e_m.dot(e_p);
                double c = e_p.dot(e_n);
                double d = e_m.dot(e_p.cross(e_n));
                theta = theta * std::complex<double>(b*c - a, d);
            }
            double arg = std::arg(theta);
            if(arg < 0){
                solid_angle = -arg;
                return -arg;
            }
            else{
                solid_angle = 2 * M_PI - arg;
                return 2 * M_PI - arg;
            }
        }
    }


private:

    void removeDuplicate(Matrix3d &E)
    {
        //remove identical edges
        stdvec_Vector3d tmp;
        for(int id = 0; id < E.cols(); id++){

            bool isDuplicate = false;
            for(int jd = 0; jd < tmp.size(); jd++)
            {
                if((tmp[jd] - E.col(id)).norm() < FLOAT_ERROR_SMALL){
                    isDuplicate = true;
                    break;
                }
            }

            if(!isDuplicate && E.col(id).norm() > FLOAT_ERROR_SMALL)
            {
                tmp.push_back(E.col(id));
                tmp.back().normalize();
            }
        }

        E = Matrix3d(3, tmp.size());
        for(int id = 0; id < tmp.size(); id++)
        {
            E.col(id) = tmp[id];
        }
    }

    void computeEdge(Matrix3d &E)
    {
        stdvec_Vector3d edges;
        //get all edges
        for(int id = 0; id < m_N.cols(); id++){
            for(int jd = id + 1; jd < m_N.cols(); jd++){
                Eigen::Vector3d e = m_N.col(id).cross(m_N.col(jd));
                if(e.norm() > FLOAT_ERROR_LARGE){
                    e.normalize();
                    edges.push_back(e);
                    edges.push_back(e * (-1));
                }
            }
        }

        //remove outside edges;
        stdvec_Vector3d tmp;
        for(int id = 0; id < edges.size(); id++)
        {
            Eigen::Vector3d e = edges[id];
            if(isEdgeValid(e))
                tmp.push_back(e);
        }
        edges = tmp;

        //put it into Matrix3f
        E = Matrix3d(3, tmp.size());
        for(int id = 0; id < tmp.size(); id++){
            E.col(id) = tmp[id];
        }

        removeDuplicate(E);
        return;
    }

    bool isEdgeValid(Eigen::Vector3d &e)
    {
        for(int id = 0; id < m_N.cols(); id++)
        {
            Eigen::Vector3d n = m_N.col(id);
            if(n.dot(e) < -FLOAT_ERROR_SMALL){
                return  false;
            }
        }
        return true;
    }

    void orderEdge(Matrix3d &E)
    {
        //finding neighborhood
        vector<vector<int>> edge2face; edge2face.resize(E.cols());
        vector<vector<int>> face2edge; face2edge.resize(m_N.cols());

        for(int id = 0; id < E.cols(); id++)
        {
            Eigen::Vector3d e = E.col(id);
            for(int jd = 0; jd < m_N.cols(); jd++)
            {
                Eigen::Vector3d n = m_N.col(jd);
                if(std::fabs(e.dot(n)) < FLOAT_ERROR_SMALL)
                {
                    edge2face[id].push_back(jd);
                    face2edge[jd].push_back(id);
                }
            }
        }

        stdvec_Vector3d edges;
        int edgeID = 0;
        map<int, bool> visited;
        do
        {
            edges.push_back(E.col(edgeID));
            visited[edgeID] = true;
            for(int fid : edge2face[edgeID]){
                for(int eid : face2edge[fid]){
                    if(visited[eid] == false){
                        edgeID = eid;
                        goto skip;
                    }
                }
            }
            skip:
            {
                //do next
            };
        }while(edges.size() < E.cols());

        for(int id = 0; id < edges.size(); id++)
        {
            E.col(id) = edges[id];
        }

        if(E.cols() > 2){
            if(E.col(0).dot(E.col(1).cross(E.col(2))) < 0)
            {
                for(int id = 0; id < edges.size(); id++)
                {
                    E.col(id) = edges[edges.size() - 1 - id];
                }
            }
        }
        return;
    }

    double angle(Eigen::Vector3d a, Eigen::Vector3d b){
        double dot = a.dot(b);
        dot = ( dot < -1.0 ? -1.0 : ( dot > 1.0 ? 1.0 : dot ) );
        return std::acos( dot );
    }
};

#endif //TOPOLOCKCREATOR_POLYHEDRALCONE_H
