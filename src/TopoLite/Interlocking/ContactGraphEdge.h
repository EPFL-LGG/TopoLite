//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
#define TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H

#include <Eigen/Dense>
#include "Utility/HelpDefine.h"
#include <Mesh/Polygon.h>

/*!
 * \brief: Stores which parts are contacted and the contact's geometry
 */
template<typename Scalar>
class ContactGraphEdge
{
public:

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    vector<pPolygon> polygons;  // a list which polygons are simple and closed (better to be convex)
    vector<Vector3> normals; // contact normal from partIDA to partIDB

public:

    ContactGraphEdge(pPolygon &_polygon, Vector3 &_normal)
    {
        polygons.push_back(_polygon);
        normals.push_back(_normal);
    }

    ContactGraphEdge(vector<pPolygon> &_polygons, vector<Vector3> &_normals)
    {
        polygons = _polygons;
        normals = _normals;
    }

    ContactGraphEdge(vector<pPolygon> &_polygons, Vector3 &_normals)
    {
        polygons = _polygons;
        for(size_t id = 0; id < polygons.size(); id++){
            normals.push_back(_normals);
        }
    }

    /*!
     * \return the contact normal starts from partID
     */
    Vector3 getContactNormal(int partID, int polyID){
        if(partIDA == partID)
        {
            return normals[polyID];
        }
        else if(partIDB == partID){
            return normals[polyID] * (-1.0f);
        }
        return Vector3(0, 0, 0);
    }

    void get_norm_fric_for_block(int partID, int polyID, Vector3 &normal, Vector3 &ufric, Vector3 &vfric)
    {
        normal = getContactNormal(partID, polyID) * (-1.0);
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
        for(pPolygon poly : polygons)
            num_vks += poly.size();
        return num_vks;
    }

public: //Automatic Generate

    int partIDA; // Static ID for start node of this edge

    int partIDB; // Static ID for end node of this edge
};



#endif //TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
