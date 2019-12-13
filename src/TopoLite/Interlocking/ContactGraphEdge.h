//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
#define TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "Utility/HelpDefine.h"

using EigenPoint =  Eigen::Vector3f;
using EigenPointList = std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f>> ;
using std::vector;

class ContactPolygon{
public:
    EigenPointList points;
    EigenPoint center;
};

/*!
 * \brief: Stores which parts are contacted and the contact's geometry
 */
class ContactGraphEdge
{
public:
    vector<ContactPolygon> polygons;  //!< a list which polygons are simple and closed (better to be convex)
    vector<EigenPoint> normals; //!< contact normal from partIDA to partIDB

public:

    ContactGraphEdge(ContactPolygon &_polygon, EigenPoint &_normal)
    {
        polygons.push_back(_polygon);
        normals.push_back(_normal);
    }

    ContactGraphEdge(vector<ContactPolygon> &_polygons, vector<EigenPoint> &_normals)
    {
        polygons = _polygons;
        normals = _normals;
    }

    ContactGraphEdge(vector<ContactPolygon> &_polygons, EigenPoint &_normals)
    {
        polygons = _polygons;
        for(int id = 0; id < polygons.size(); id++){
            normals.push_back(_normals);
        }
    }

    /*!
     * \return the contact normal starts from partID
     */
    EigenPoint getContactNormal(int partID, int polyID){
        if(partIDA == partID)
        {
            return normals[polyID];
        }
        else if(partIDB == partID){
            return normals[polyID] * (-1.0f);
        }
        return EigenPoint();
    }

    void get_norm_fric_for_block(int partID, int polyID, Eigen::Vector3d &normal, Eigen::Vector3d & ufric, Eigen::Vector3d & vfric)
    {
        normal = getContactNormal(partID, polyID).cast<double>() * (-1.0);
        ufric = Eigen::Vector3d(1, 0, 0).cross(normal);
        if((ufric).norm() < FLOAT_ERROR_SMALL){
            ufric = Eigen::Vector3d(0, 1, 0).cross(normal);
        }
        ufric /= ufric.norm();
        vfric = normal.cross(ufric); vfric /= vfric.norm();
        return;
    }

    int num_points()
    {
        int num_vks = 0;
        for(ContactPolygon poly : polygons)
            num_vks += poly.points.size();
        return num_vks;
    }

public: //Automatic Generate

    int partIDA; //!< Static ID for start node of this edge

    int partIDB; //!< Static ID for end node of this edge
};



#endif //TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
