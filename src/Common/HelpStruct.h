/*!
 * \file HelpStruc.h
 * \brief HelpStruc.h includes all basic geometric data structure
 *
 * List of all class names: 3D Point, 3D Line, 3D Box, 3D Plane, 3D Triangle
 *
 */

#ifndef TOPOLOCKCREATOR2_HELPSTRUC_H
#define TOPOLOCKCREATOR2_HELPSTRUC_H

#include <Eigen/Dense>
#include <vector>
#include "HelpDefine.h"

//! Check if a point is on a plane
const double SMALL_ERROR  = 0.000001;

//! Point position w.r.t a plane
#define POINT_PLANE_UNKWONN          -1
#define POINT_PLANE_INTERSECT         0
#define POINT_PLANE_POSITIVE_SIDE     1
#define POINT_PLANE_NEGATIVE_SIDE     2

//! Line position w.r.t a plane
#define LINE_PLANE_UNKWONN           -1
#define LINE_PLANE_INTERSECT          0
#define LINE_PLANE_POSITIVE_SIDE      1
#define LINE_PLANE_NEGATIVE_SIDE      2

//! Face position w.r.t a plane
#define FACE_PLANE_UNKWONN           -1
#define FACE_PLANE_INTERSECT          0
#define FACE_PLANE_POSITIVE_SIDE      1
#define FACE_PLANE_NEGATIVE_SIDE      2

//! Whether an element is outside of a list
#define ELEMENT_OUT_LIST -1

using Eigen::Vector3i;
using Eigen::Vector3d;
using Eigen::Vector2i;
using Eigen::Vector2d;

/*!
 * \file HelpStruct.h
 * \class Point
 * \brief Point position and other attributes
 */
struct Point
{
    Vector3d pos;     //!< Point position
    Vector3d nor;     //!< Point normal

    Vector3d color;   //!< Point color (if available)
    double curv;       //!< Point curvature

    int dist;         //!< Distance to the object surface (if available)

    /*!
    * \brief Copy and Construction Fuction
    */
    Point & operator=(const Point &pt)
    {
        if( this == &pt ) return *this;

        this->pos   = pt.pos;
        this->nor   = pt.nor;

        this->color = pt.color;
        this->curv  = pt.curv;

        this->dist = pt.dist;

        return *this;
    };
};



/*!
 * \file HelpStruct.h
 * \class Line
 * \brief Line = Point1 -> Point2
 */
struct Line
{
    Vector3d point1; //!< start point
    Vector3d point2; //!< end point
};


/*!
 * \file HelpStruct.h
 * \class Plane
 * \brief Store the plane geometry and compute the intersection with plane or line.
 */
struct Plane
{
public:

    Vector3d point;  //!< point on the plane
    Vector3d normal; //!< normal of the plane

    double radius;    //!< For define a limited plane in 3D (for rendering)

public:

    Plane & operator=(const Plane &plane); //!< Copy and Construct Function

public:

    /*!
     * \brief Compute the distance from the tagtPt to this plane
     * \param tagtPt
     * \return the distance from the \b tagtPt to this plane.
     */
    double PointPlaneDistance(Vector3d tagtPt);

    /*!
     * \param tagtPt the point, isPrint printing debug information
     * \return a index (LINE_PLANE_UNKWONN, POINT_PLANE_INTERSECT, POINT_PLANE_POSITIVE_SIDE, POINT_PLANE_NEGATIVE_SIDE)
     * \brief Check whether the point is on this plane <br>
     *  LINE_PLANE_UNKWONN: unknow <br>
     *  POINT_PLANE_INTERSECT: point on the plane <br>
     *  POINT_PLANE_POSITIVE_SIDE: point on the side which normal points to <br>
     *  POINT_PLANE_NEGATIVE_SIDE: point on the side which normal does not point to <br>
     */
    int PointPlaneIntersect(Vector3d tagtPt, bool isPrint);

    int LinePlaneIntersect(Line line);

    int LineIntersectPoint(Line line, Vector3d &crossPt);

    void DrawFace(double radius, Vector3d color);
};


////////////////////////////////////////////
// 3D Box
////////////////////////////////////////////

struct Box
{
    Vector3d minPt;
    Vector3d maxPt;

    Vector3d cenPt;
    Vector3d size;

    Box();
    Box & operator=(const Box &box);
    void PrintBox();

    void GetCenter();
    void GetSize();

    void Transform(Vector3d transVec, Vector3d scale);
    double GetQuadArea();
};


////////////////////////////////////////////
// 3D Triangle
////////////////////////////////////////////

struct Triangle
{
    Vector3d v[3];
    Vector3d normal;
    double area;            // Face area
    Vector3d center;

    int vIndices[3];       // Index of each vertex


    void Init(Vector3d _v0, Vector3d _v1, Vector3d _v2);
    Triangle & operator=(const Triangle &tri);
    bool IsEqual(Triangle *tri);
    void PrintTriangle();

    Vector3d GetBBoxMinPt();
    Vector3d GetBBoxMaxPt();

    void ComputeCenter();
    void ComputeArea();
    void ComputeNormal();
    void CorrectNormal(Vector3d tagtNormal);

    void Draw();
    void DrawWire(double width, Vector3d color);
};

#endif //TOPOLOCKCREATOR2_HELPSTRUC_H