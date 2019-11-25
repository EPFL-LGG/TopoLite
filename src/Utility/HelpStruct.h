///////////////////////////////////////////////////////////////
//
// Utility/HelpStruct.h
//
//   Common Structures
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef _HELP_STRUCT_H
#define _HELP_STRUCT_H

#include "vec.h"
#include <vector>
#include "Utility/Controls.h"

using namespace std;

// Check if a point is on a plane
#define SMALL_ERROR                  0.000001     

// Point position w.r.t a plane
#define POINT_PLANE_UNKWONN          -1
#define POINT_PLANE_INTERSECT         0
#define POINT_PLANE_POSITIVE_SIDE     1
#define POINT_PLANE_NEGATIVE_SIDE     2

// Line position w.r.t a plane
#define LINE_PLANE_UNKWONN           -1
#define LINE_PLANE_INTERSECT          0
#define LINE_PLANE_POSITIVE_SIDE      1
#define LINE_PLANE_NEGATIVE_SIDE      2

// Face position w.r.t a plane
#define FACE_PLANE_UNKWONN           -1
#define FACE_PLANE_INTERSECT          0
#define FACE_PLANE_POSITIVE_SIDE      1
#define FACE_PLANE_NEGATIVE_SIDE      2

////////////////////////////////////////////
// 3D Point
////////////////////////////////////////////

// 3D Point (with normal) 
struct Point
{
	Vector3f pos;     // Point position
	Vector3f nor;     // Point normal

	Vector3f color;   // Point color (if available)
	float curv;       // Point curvature

	int dist;         // Distance to the object surface

	Point & operator=(const Point &pt)
	{
		if( this == &pt )
			return *this;

		this->pos   = pt.pos;
		this->nor   = pt.nor;

		this->color = pt.color;
		this->curv  = pt.curv;

		return *this;
	};
};


////////////////////////////////////////////
// 3D Line
////////////////////////////////////////////

struct Line
{
	Vector3f point1;
	Vector3f point2;
};


////////////////////////////////////////////
// 3D Plane
////////////////////////////////////////////

struct Plane
{
	Vector3f point;
	Vector3f normal;

	float radius;             // For define a limited plane in 3D

	Plane & operator=(const Plane &plane);

	float PointPlaneDistance(Vector3f tagtPt);

	int PointPlaneIntersect(Vector3f tagtPt, bool isPrint);
	int LinePlaneIntersect(Line line);
	int LineIntersectPoint(Line line, Vector3f &crossPt);
#if USE_OPENGL_DRAW
	void DrawFace(float radius, Vector3f color);
#endif
};

////////////////////////////////////////////
// 3D Box
////////////////////////////////////////////

struct Box 
{
	Vector3f minPt;
	Vector3f maxPt;

	Vector3f cenPt;
	Vector3f size;

	Box();
	Box & operator=(const Box &box);
	void PrintBox();

	void GetCenter();
	void GetSize();

	void Transform(Vector3f transVec, Vector3f scale);
	float GetQuadArea();
};


////////////////////////////////////////////
// 3D Triangle
////////////////////////////////////////////

struct Triangle
{
	Vector3f v[3];
	Vector3f normal;
	float area;            // Face area
	Vector3f center;

	int vIndices[3];       // Index of each vertex


	void Init(Vector3f _v0, Vector3f _v1, Vector3f _v2);
	Triangle & operator=(const Triangle &tri);
	bool IsEqual(Triangle *tri);
	void PrintTriangle();

	Vector3f GetBBoxMinPt();
	Vector3f GetBBoxMaxPt();

	void ComputeCenter();
	void ComputeArea();
	void ComputeNormal();
	void CorrectNormal(Vector3f tagtNormal);

#if USE_OPENGL_DRAW
	void Draw();
	void DrawWire(float width, Vector3f color);
#endif
};


////////////////////////////////////////////
// For computing part geometry and mobility
////////////////////////////////////////////

struct OrientPoint
{

	/*for geometry generation*/
	Vector3f point;
	Vector3f normal;
    Vector3f rotation_axis;
    Vector3f rotation_base;

	/*for optimization*/
	float rotation_angle;
	int tiltSign;              // Flag that indicates the direction to tilt the normal (possible values are {-1, 1})
    int oriptID;               //the index of the oriented point in the whole structure
	Vector2f tilt_range;	   //the lower and upper bound of tilt angle such that the structure have valid geometry.
	Vector2f sided_range;	   //for debug, show the side range
public:

	OrientPoint(Vector3f _point, Vector3f _normal)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = Vector3f(0, 0, 0);
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range.x = 0;
		tilt_range.y = 180;
	}

	OrientPoint(Vector3f _point, Vector3f _normal, Vector3f _axis)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = _axis;
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range.x = 0;
		tilt_range.y = 180;
	};

	void update_rotation(float _angle)
	{
		rotation_angle = _angle;
	}

	void Print()
	{
		printf("point: [%6.3f %6.3f %6.3f]   normal: [%6.3f %6.3f %6.3f] \n", point.x, point.y, point.z, normal.x, normal.y, normal.z);
		printf("rotation_axis: [%6.3f %6.3f %6.3f]   rotation_angle: [%6.3f] \n", rotation_axis.x, rotation_axis.y, rotation_axis.z, rotation_angle);
	};
};

struct HypVertex
{
public:
	int verID;
	Vector3f point;

	//int planeIDs[3];
	int edgeID;         // For part mobility
	bool isValid;
};

struct HypEdge
{
public:
	int edgeID;
	Vector3f point;
	Vector3f normal;

	int planeIDs[2];
};

struct HypPlane
{
public:
	int planeID;
	Vector3f point;
	Vector3f normal;

public:
	double getD(){return normal DOT point;}

	float radius;             // For rendering a finite plane in 3D
};


#endif