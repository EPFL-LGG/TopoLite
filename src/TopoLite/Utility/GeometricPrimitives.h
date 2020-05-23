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

#ifndef GeometricPrimitives_H
#define GeometricPrimitives_H

#include "TopoLite/Utility/HelpDefine.h"

#include <vector>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Vector3f;

using namespace std;

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
template <typename Scalar>
struct Point
{
    typedef Matrix<Scalar,3, 1> Vector3;

    Vector3 pos;     // Point position
    Vector3 nor;     // Point normal
    Vector3 color;   // Point color (if available)

    Scalar curv;       // Point curvature
	int dist;         // Distance to the object surface

	Point(){
	    pos = Vector3(0, 0, 0);
        nor = Vector3(0, 0, 0);
        color = Vector3(0, 0, 0);
        curv = 0;
        dist = 0;
	}

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
// 3D Vertex
////////////////////////////////////////////

template <typename Scalar>
class VPoint
{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;

public:

    Vector3 pos;				// vertex's postion (X,Y,Z)

    int verID;

public:
    VPoint()
    {
        verID = -1;
    };

    VPoint(const Vector3 &_pos)
    {
        pos   = _pos;
        verID = -1;
    };
};


template <typename Scalar>
class VTex
{
public:

    typedef Matrix<Scalar, 2 ,1> Vector2;

public:

    Vector2 texCoord;				// vertex's postion (X,Y,Z)

    int texID;

public:
    VTex()
    {
        texID = -1;
    };

    VTex(const Vector2& _texCoord)
    {
        texCoord = _texCoord;
        texID = -1;
    };
};


////////////////////////////////////////////
// 3D Line
////////////////////////////////////////////

template <typename Scalar>
struct Line
{
    typedef Matrix<Scalar,3, 1> Vector3;
    typedef Matrix<Scalar,2, 1> Vector2;

    Vector3 point1;
	Vector3 point2;

	Line(){
	    point1 = Vector3(0 ,0 ,0);
        point2 = Vector3(0 ,0 ,0);
	}

	Line(Vector3 pt1, Vector3 pt2){
	    point1 = pt1;
	    point2 = pt2;
	}

    Line(Vector2 pt1, Vector2 pt2){
        point1 = Vector3(pt1.x(), pt1.y(), 0);
        point2 = Vector3(pt2.x(), pt2.y(), 0);
    }
};


////////////////////////////////////////////
// 3D Plane
////////////////////////////////////////////

template <typename Scalar>
struct Plane
{
    typedef Matrix<Scalar,3, 1> Vector3;

    Vector3 point;
	Vector3 normal;

    Plane(){
        point = Vector3(0, 0, 0);
        normal = Vector3(0, 0, 0);
    }

	Plane & operator=(const Plane &plane);

	Scalar computePtPtDistance(Vector3 tagtPt);
	int computePtPlnIntersec(Vector3 tagtPt);

	int checkLnPlnIntersec(Line<Scalar> line);
	int computeLnLnIntersec(Line<Scalar> line, Vector3 &crossPt);
};

template<typename Scalar>
Plane<Scalar> & Plane<Scalar>::operator=(const Plane<Scalar> &plane)
{
    if( this == &plane )
        return *this;

    this->point  = plane.point;
    this->normal = plane.normal;

    this->radius = plane.radius;

    return *this;
}

template<typename Scalar>
Scalar Plane<Scalar>::computePtPtDistance(Vector3 tagtPt)
{
    Vector3 tagtvec = tagtPt - point;

    Scalar dotP = tagtvec.dot(normal);
    Scalar dist = std::abs(dotP);

    return dist;
}


template<typename Scalar>
int Plane<Scalar>::computePtPlnIntersec(Vector3 tagtPt)
{
    Vector3 vec = tagtPt - point;
    Scalar dotP = normal.dot(vec);

    if ( std::abs(dotP) < FLOAT_ERROR_SMALL)
        return POINT_PLANE_INTERSECT;       // Note: may need to tune this small threshold
    else if ( dotP >  0 )
        return POINT_PLANE_POSITIVE_SIDE;
    else if ( dotP <  0 )
        return POINT_PLANE_NEGATIVE_SIDE;

    return POINT_PLANE_UNKWONN;
}

template<typename Scalar>
int Plane<Scalar>::checkLnPlnIntersec(Line<Scalar> line)
{
    int pt1State = computePtPlnIntersec(line.point1);
    int pt2State = computePtPlnIntersec(line.point2);

    if ( pt1State == POINT_PLANE_POSITIVE_SIDE &&
         pt2State == POINT_PLANE_POSITIVE_SIDE )
        return LINE_PLANE_POSITIVE_SIDE;

    else
    if ( pt1State == POINT_PLANE_NEGATIVE_SIDE &&
         pt2State == POINT_PLANE_NEGATIVE_SIDE )
        return LINE_PLANE_NEGATIVE_SIDE;

    else
        return LINE_PLANE_INTERSECT;
}

template<typename Scalar>
int Plane<Scalar>::computeLnLnIntersec(Line<Scalar> line, Vector3 &crossPt)
{
    // Check if the line intersects the plane
    int state = checkLnPlnIntersec( line );
    if ( state != LINE_PLANE_INTERSECT )
    {
        //printf("Warning: Line does not intersect with the plane. \n");
        return state;
    }

    // Compute the intersected point between the line and the plane
    Vector3 rayOrg = line.point1;
    Vector3 rayDir = (line.point2-line.point1) / (line.point2-line.point1).norm();
    Vector3 tempVec = point - rayOrg;
    Scalar m = (normal.dot(tempVec)) / (normal.dot(rayDir));

    crossPt[0] = rayOrg[0] + m * rayDir[0];
    crossPt[1] = rayOrg[1] + m * rayDir[1];
    crossPt[2] = rayOrg[2] + m * rayDir[2];

    return state;
}


////////////////////////////////////////////
// 3D Box
////////////////////////////////////////////

template <typename Scalar>
struct Box 
{
    typedef Matrix<Scalar,3, 1> Vector3;

public:
    //storage
    Vector3 minPt;
    Vector3 maxPt;

public:
    //compute
    Vector3 cenPt;
    Vector3 size;

public:
	Box();
	Box(const Line<Scalar> &line);
	Box(const Box<Scalar> &b0, const Box<Scalar> &b1);
	Box & operator=(const Box &box);
	void print();

	void computeCenter();
	void computeSize();

	void executeTransform(Vector3 transVec, Vector3 scale);

	//if this box degenerates into a quad plane, compute its area.
	Scalar computeQuadArea();
};

template<typename Scalar>
Box<Scalar>::Box()
{
    minPt = Vector3(0, 0, 0);
    maxPt = Vector3(0, 0, 0);
    cenPt = Vector3(0, 0, 0);
    size = Vector3(0, 0, 0);
}

template<typename Scalar>
Box<Scalar>::Box(const Line<Scalar> &line)
{
    for(size_t id = 0; id < 3; id++){
        minPt[id] = std::min(line.point1[id], line.point2[id]);
        maxPt[id] = std::max(line.point1[id], line.point2[id]);
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}

template<typename Scalar>
Box<Scalar>::Box(const Box<Scalar> &b0, const Box<Scalar> &b1){

    for(size_t id = 0; id < 3; id++)
    {
        minPt[id] = std::min(b0.minPt[id], b1.minPt[id]);
        maxPt[id] = std::max(b0.maxPt[id], b1.maxPt[id]);
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}


template<typename Scalar>
Box<Scalar> & Box<Scalar>::operator=(const Box &box)
{
    if( this == &box )
        return *this;

    this->minPt = box.minPt;
    this->maxPt = box.maxPt;
    this->cenPt = box.cenPt;

    return *this;
}

template<typename Scalar>
void Box<Scalar>::print()
{
    printf("Box: [%7.3f  %7.3f  %7.3f]      [%7.3f  %7.3f  %7.3f] \n", minPt.x(), minPt.y(), minPt.z(), maxPt.x(), maxPt.y(), maxPt.z());
}

template<typename Scalar>
void Box<Scalar>::computeCenter()
{
    cenPt = 0.5f * ( minPt + maxPt );
}

template<typename Scalar>
void Box<Scalar>::computeSize()
{
    size = maxPt - minPt;
}

template<typename Scalar>
void Box<Scalar>::executeTransform(Vector3 transVec, Vector3 scale)
{
    minPt.x() *= scale.x();  minPt.y() *= scale.y();  minPt.z() *= scale.z();
    maxPt.x() *= scale.x();  maxPt.y() *= scale.y();  maxPt.z() *= scale.z();
    cenPt.x() *= scale.x();  cenPt.y() *= scale.y();  cenPt.z() *= scale.z();
    size.x() *= scale.x();   size.y() *= scale.y();   size.z() *= scale.z();

    minPt += transVec;
    maxPt += transVec;
    cenPt += transVec;
}

template<typename Scalar>
Scalar Box<Scalar>::computeQuadArea()
{
    Vector3 dimen = maxPt - minPt;
    Scalar quadArea = 0;

    if      ( std::abs(dimen.x()) < FLOAT_ERROR_SMALL && dimen.y()  > 0 &&  dimen.z()  > 0 )
        quadArea = dimen.y() * dimen.z();  // y-z plane quad
    else if ( dimen.x()  > 0 && std::abs(dimen.y()) < FLOAT_ERROR_SMALL &&  dimen.z()  > 0 )
        quadArea = dimen.x() * dimen.z();  // x-z plane quad
    else if ( dimen.x()  > 0 && dimen.y()  > 0 &&  std::abs(dimen.z()) < FLOAT_ERROR_SMALL )
        quadArea = dimen.x() * dimen.y();  // x-y plane quad
    else
        printf("Warning: The box is not degenerated into a quad. \n");

    return quadArea;
}


////////////////////////////////////////////
// 3D Triangle
////////////////////////////////////////////

template <typename Scalar>
struct Triangle
{
public:
    typedef Matrix<Scalar,3, 1> Vector3;

    typedef Matrix<Scalar,2, 1> Vector2;

public:

    Vector3 v[3];

	int vIndices[3];       // Index of each vertex

	bool edge_at_boundary[3];
public:
    Triangle(){
        v[0] = Vector3(0, 0, 0);
        v[1] = Vector3(0, 0, 0);
        v[2] = Vector3(0, 0, 0);
        edge_at_boundary[0] = false;
        edge_at_boundary[1] = false;
        edge_at_boundary[2] = false;
    }

    Triangle(Vector3 _v0, Vector3 _v1, Vector3 _v2)
    {
        v[0] = _v0;
        v[1] = _v1;
        v[2] = _v2;
        edge_at_boundary[0] = false;
        edge_at_boundary[1] = false;
        edge_at_boundary[2] = false;
    }

    Triangle(Vector2 _v0, Vector2 _v1, Vector2 _v2)
    {
        v[0].x() = _v0.x(); v[0].y() = _v0.y(); v[0].z() = 0;
        v[1].x() = _v1.x(); v[1].y() = _v1.y(); v[1].z() = 0;
        v[2].x() = _v2.x(); v[2].y() = _v2.y(); v[2].z() = 0;
        edge_at_boundary[0] = false;
        edge_at_boundary[1] = false;
        edge_at_boundary[2] = false;
    }

    Triangle(Vector3 _v0, Vector3 _v1, Vector3 _v2, bool b0, bool b1, bool b2)
    {
        v[0] = _v0;
        v[1] = _v1;
        v[2] = _v2;
        edge_at_boundary[0] = b0;
        edge_at_boundary[1] = b1;
        edge_at_boundary[2] = b2;
    }

public:

	void init(Vector3 _v0, Vector3 _v1, Vector3 _v2);

	Triangle & operator=(const Triangle &tri);

	bool checkEqual(const Triangle<Scalar> &tri);

	void print();

	Vector3 computeBBoxMinPt();

	Vector3 computeBBoxMaxPt();

    Vector3 computeCenter();

	Scalar computeArea();

    Scalar computeSignedArea(); //for 2D triangle

	Vector3 computeNormal();

    void correctNormal(Vector3 tagtNormal);
};

template<typename Scalar>
void Triangle<Scalar>::init(Vector3 _v0, Vector3 _v1, Vector3 _v2)
{
    v[0] = _v0;
    v[1] = _v1;
    v[2] = _v2;
}

template<typename Scalar>
Triangle<Scalar> & Triangle<Scalar>::operator=(const Triangle &tri)
{
    if( this == &tri )
        return *this;

    for (int i=0; i<3; i++)
    {
        this->v[i] = tri.v[i];
        this->vIndices[i] = tri.vIndices[i];
    }
    return *this;
}

template<typename Scalar>
bool Triangle<Scalar>::checkEqual(const Triangle<Scalar> &tri)
{
    if( this->v[0] == tri.v[0] &&
        this->v[1] == tri.v[1] &&
        this->v[2] == tri.v[2] )
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<typename Scalar>
void Triangle<Scalar>::print()
{
    printf("v0: [%.12f %.12f %.12f] \n", v[0].x(), v[0].y(), v[0].z());
    printf("v1: [%.12f %.12f %.12f] \n", v[1].x(), v[1].y(), v[1].z());
    printf("v2: [%.12f %.12f %.12f] \n", v[2].x(), v[2].y(), v[2].z());
    printf("\n");
}

template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeBBoxMinPt()
{
    Vector3 bboxMinPt;

    bboxMinPt.x() = _MIN(v[0].x(), _MIN(v[1].x(), v[2].x()));
    bboxMinPt.y() = _MIN(v[0].y(), _MIN(v[1].y(), v[2].y()));
    bboxMinPt.z() = _MIN(v[0].z(), _MIN(v[1].z(), v[2].z()));

    return bboxMinPt;
}
template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeBBoxMaxPt()
{
    Vector3 bboxMaxPt;

    bboxMaxPt.x() = _MAX(v[0].x(), _MAX(v[1].x(), v[2].x()));
    bboxMaxPt.y() = _MAX(v[0].y(), _MAX(v[1].y(), v[2].y()));
    bboxMaxPt.z() = _MAX(v[0].z(), _MAX(v[1].z(), v[2].z()));

    return bboxMaxPt;
}

template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeCenter()
{
    Vector3 center = (v[0] + v[1] + v[2]) / 3.0;
    return center;
}

template<typename Scalar>
Scalar Triangle<Scalar>::computeArea()
{
    Vector3 normal  = (v[1] - v[0]).cross(v[2] - v[0]);
    Scalar area  = 0.5f * normal.norm();
    return area;
}

//https://www.mn.uio.no/math/english/people/aca/michaelf/papers/wach_mv.pdf
template<typename Scalar>
Scalar Triangle<Scalar>::computeSignedArea()
{
    Matrix<Scalar, 3, 3> A;
    A << 1, 1, 1,
    v[0].x(), v[1].x(), v[2].x(),
    v[0].y(), v[1].y(), v[2].y();
    return A.determinant() / 2;
}


template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeNormal()
{
    Vector3 tempNor = (v[1] - v[0]).cross(v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
    Scalar tempNorLen = tempNor.norm();
    Vector3 normal;
    if ( tempNorLen > FLOAT_ERROR_SMALL )
    {
        normal = tempNor / tempNorLen;
    }
    else{
        normal = Vector3(1,0,0);     // Note: this default vector also can be others
    }
    return normal;
}

template<typename Scalar>
void Triangle<Scalar>::correctNormal(Vector3 tagtNormal)
{
    // Compute initial normal
    Vector3 normal = computeNormal();

    // Rearrange vertex order if needed
    Scalar dotp = normal.dot(tagtNormal);
    if ( dotp < 0 )
    {
        Vector3 triVers[3];
        for (int i=0; i<3; i++)
        {
            triVers[i] = v[i];
        }

        v[0] = triVers[0];
        v[1] = triVers[2];
        v[2] = triVers[1];
    }
}

////////////////////////////////////////////
// For computing part geometry and mobility
////////////////////////////////////////////

template <typename Scalar>
struct OrientPoint
{
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, 2, 1> Vector2;

    // for geometry generation
    Vector3 point;
    Vector3 normal;             // normed vector
    Vector3 rotation_axis;
    Vector3 rotation_base;

	// for optimization
	Scalar rotation_angle;     // always positive
	int tiltSign;              // Flag that indicates the direction to tilt the normal (possible values are {-1, 1})
    int oriptID;               // the index of the oriented point in the whole structure
    Vector2 tilt_range;	       // the lower and upper bound of tilt angle such that the structure have valid geometry.
    Vector2 sided_range;	   // for debug, show the side range
public:

	OrientPoint(Vector3 _point, Vector3 _normal)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = Vector3(0, 0, 0);
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range[0] = 0;
		tilt_range[1] = 180;
	}

	OrientPoint(Vector3 _point, Vector3 _normal, Vector3 _axis)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = _axis;
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range[0] = 0;
		tilt_range[1] = 180;
	};

    /**
        @brief: Compute vector rotation
        @param[in] normal based vector
        @param[in] rotAxis rotation axis
        @param[in] rotAngle rotation angle
        @return vector after rotation
    */
    static Vector3 rotateVecAroundAxis(Vector3 normal, Vector3 rotAxis, Scalar rotAngle)
    {
        rotAngle = rotAngle / 180 * M_PI;
        Eigen::Matrix<Scalar, 3, 3> rotationMat, crossprodMat;
        crossprodMat << 0,             -rotAxis.z(),   rotAxis.y(),
                        rotAxis.z(),    0,            -rotAxis.x(),
                       -rotAxis.y(),    rotAxis.x(),   0;
        rotationMat = Eigen::Matrix<Scalar, 3, 3>::Identity() * std::cos(rotAngle) + crossprodMat * std::sin(rotAngle);
        return rotationMat * normal;
    }

    /**
        Update rotation_angle (always positive) + define normal
    */
    void updateAngle(Scalar _angle)
	{
		rotation_angle = std::abs(_angle);
        normal = rotateVecAroundAxis(rotation_base, rotation_axis, rotation_angle * tiltSign);
	}

	void print()
	{
		printf("point: [%6.3f %6.3f %6.3f]   normal: [%6.3f %6.3f %6.3f] \n", point[0], point[1], point[2], normal[0], normal[1], normal[2]);
		printf("rotation_axis: [%6.3f %6.3f %6.3f]   rotation_angle: [%6.3f] \n", rotation_axis[0], rotation_axis[1], rotation_axis[2], rotation_angle);
	};
};

template <typename Scalar>
struct HypVertex
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int verID;
	Vector3 point;
	int planeIDs[3];
};

template <typename Scalar>
struct HypEdge
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int edgeID;
	Vector3 point;
	Vector3 normal;

	int planeIDs[2];
};

template <typename Scalar>
struct HypPlane
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int planeID;
	Vector3 point;
	Vector3 normal;

public:
	double getD(){return normal.dot(point);}

	Scalar radius;             // For rendering a finite plane in 3D
};


#endif