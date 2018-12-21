///////////////////////////////////////////////////////////////
//
// HelpStruct.cpp
//
//   Common Structures
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
//
///////////////////////////////////////////////////////////////

#ifdef WIN32
#include <GL/glut.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif // __APPLE__

#include "HelpStruct.h"
#include "HelpDefine.h"


//**************************************************************************************//
//                                   3D Plane
//**************************************************************************************//

Plane & Plane::operator=(const Plane &plane)
{
	if( this == &plane )
		return *this;

	this->point  = plane.point;
	this->normal = plane.normal;

	this->radius = plane.radius;

	return *this;
}

double Plane::PointPlaneDistance(Vector3d tagtPt)
{
	Vector3d tagtvec = tagtPt - point;

	double dotP = tagtvec.dot(normal);
	double dist = fabs( dotP );

	return dist;
}


int Plane::PointPlaneIntersect(Vector3d tagtPt, bool isPrint)
{
	Vector3d vec = tagtPt - point;
	double dotP = normal.dot(vec);

	if ( isPrint )
	{
		printf("dotp: %.8f \n", dotP);
	}

	if ( fabs(dotP) < SMALL_ERROR )    return POINT_PLANE_INTERSECT;       // Note: may need to tune this small threshold
	else if ( dotP >  0 )              return POINT_PLANE_POSITIVE_SIDE;
	else if ( dotP <  0 )              return POINT_PLANE_NEGATIVE_SIDE; 

	return POINT_PLANE_UNKWONN;
}

int Plane::LinePlaneIntersect(Line line)
{
	int pt1State = PointPlaneIntersect( line.point1, false );
	int pt2State = PointPlaneIntersect( line.point2, false );

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

int Plane::LineIntersectPoint(Line line, Vector3d &crossPt)
{
	// Check if the line intersects the plane
	int state = LinePlaneIntersect( line );
	if ( state != LINE_PLANE_INTERSECT )
	{
		//printf("Warning: Line does not intersect with the plane. \n");
		return state;
	}

	// Compute the intersected point between the line and the plane
	Vector3d rayOrg = line.point1;
	Vector3d rayDir = (line.point2-line.point1) / (line.point2-line.point1).norm();
	Vector3d tempVec = point - rayOrg;
	double m = (normal.dot(tempVec)) / (normal.dot(rayDir));

	crossPt[0] = rayOrg[0] + m * rayDir[0];
	crossPt[1] = rayOrg[1] + m * rayDir[1];
	crossPt[2] = rayOrg[2] + m * rayDir[2];

	return state;
}

// TODO: refine this function and remove duplicated ones
void Plane::DrawFace(double radius, Vector3d color)
{
	Vector3d linePt = point + 0.3f * normal; 

	//Vector3d localX = normal CROSS Vector3d(0,0,1); 
	Vector3d localX = normal.cross(Vector3d(0.5,0.5,0.5));
	Vector3d localY = normal.cross(localX);

	normal = normal / (normal).norm();
	localX = localX / (localX).norm();
	localY = localY / (localY).norm();

	//printf("N: [%.2f %.2f %.2f] \n", normal[0], normal[1], normal[2]);
	//printf("X: [%.2f %.2f %.2f] \n", localX[0], localX[1], localX[2]);
	//printf("Y: [%.2f %.2f %.2f] \n", localY[0], localY[1], localY[2]);

	glDisable(GL_LIGHTING);
	glPointSize(12.0);
	glLineWidth(3.0);

	// Draw the plane point
	glColor3f(0.9,0.6,0.3);
	glBegin(GL_POINTS);
	glVertex3f(point[0], point[1], point[2]);
	glEnd();

	// Draw the plane normal
	glColor3f(0.9,0.6,0.3);
	glBegin(GL_LINES);
	glVertex3f(point[0],  point[1],  point[2]);
	glVertex3f(linePt[0], linePt[1], linePt[2]);
	glEnd();

	// Draw the plane disk
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_POLYGON);
	int polyPtNum = 20;
	for (int i=0; i<polyPtNum; i++)
	{
		double theta = i* 2*M_PI / (double)polyPtNum;
		Vector3d polyPt = point + radius*(localX*cos(theta) + localY*sin(theta));
		glVertex3f(polyPt[0], polyPt[1], polyPt[2]);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glPointSize(1.0);
	glLineWidth(1.0);
}




//**************************************************************************************//
//                                     3D Box
//**************************************************************************************//

Box::Box()
{
	minPt = Vector3d(0, 0, 0);
	maxPt = Vector3d(0, 0, 0);
	cenPt = Vector3d(0, 0, 0);
}

Box & Box::operator=(const Box &box)
{
	if( this == &box )
		return *this;

	this->minPt = box.minPt;
	this->maxPt = box.maxPt;
	this->cenPt = box.cenPt;

	return *this;
}

void Box::PrintBox()
{
	printf("Box: [%7.3f  %7.3f  %7.3f]      [%7.3f  %7.3f  %7.3f] \n", minPt[0], minPt[1], minPt[2], maxPt[0], maxPt[1], maxPt[2]);
}


void Box::GetCenter()
{
	cenPt = 0.5f * ( minPt + maxPt );
}

void Box::GetSize()
{
	size = maxPt - minPt;
}

void Box::Transform(Vector3d transVec, Vector3d scale)
{
	minPt[0] *= scale[0];  minPt[1] *= scale[1];  minPt[2] *= scale[2];
	maxPt[0] *= scale[0];  maxPt[1] *= scale[1];  maxPt[2] *= scale[2];
	cenPt[0] *= scale[0];  cenPt[1] *= scale[1];  cenPt[2] *= scale[2];

	minPt += transVec;
	maxPt += transVec;
	cenPt += transVec;
}

double Box::GetQuadArea()
{
	Vector3d dimen = maxPt - minPt;
	double quadArea = 0;

	if      ( dimen[0] == 0 && dimen[1]  > 0 &&  dimen[2]  > 0 )    quadArea = dimen[1] * dimen[2];  // y-z plane quad
	else if ( dimen[0]  > 0 && dimen[1] == 0 &&  dimen[2]  > 0 )    quadArea = dimen[0] * dimen[2];  // x-z plane quad
	else if ( dimen[0]  > 0 && dimen[1]  > 0 &&  dimen[2] == 0 )    quadArea = dimen[0] * dimen[1];  // x-y plane quad
	else                                                         printf("Warning: The box is not degenerated into a quad. \n");
		
	return quadArea;
}




//**************************************************************************************//
//                                    3D Triangle
//**************************************************************************************//

void Triangle::Init(Vector3d _v0, Vector3d _v1, Vector3d _v2)
{
	v[0] = _v0;
	v[1] = _v1;
	v[2] = _v2;
}

Triangle & Triangle::operator=(const Triangle &tri)
{
	if( this == &tri )
		return *this;

	for (int i=0; i<3; i++)
	{
		this->v[i] = tri.v[i];
		this->vIndices[i] = tri.vIndices[i];
	}

	this->normal = tri.normal;
	this->center = tri.center;
	this->area   = tri.area;

	return *this;
}

bool Triangle::IsEqual(Triangle *tri)
{
	if( this->v[0] == tri->v[0] &&
		this->v[1] == tri->v[1] &&
		this->v[2] == tri->v[2] )
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Triangle::PrintTriangle()
{
	printf("v0: [%.12f %.12f %.12f] \n", v[0][0], v[0][1], v[0][2]);
	printf("v1: [%.12f %.12f %.12f] \n", v[1][0], v[1][1], v[1][2]);
	printf("v2: [%.12f %.12f %.12f] \n", v[2][0], v[2][1], v[2][2]);
	printf("\n");
}

Vector3d Triangle::GetBBoxMinPt()
{
	Vector3d bboxMinPt;

	bboxMinPt[0] = _MIN(v[0][0], _MIN(v[1][0], v[2][0]));
	bboxMinPt[1] = _MIN(v[0][1], _MIN(v[1][1], v[2][1]));
	bboxMinPt[2] = _MIN(v[0][2], _MIN(v[1][2], v[2][2]));

	return bboxMinPt;
}

Vector3d Triangle::GetBBoxMaxPt()
{
	Vector3d bboxMaxPt;

	bboxMaxPt[0] = _MAX(v[0][0], _MAX(v[1][0], v[2][0]));
	bboxMaxPt[1] = _MAX(v[0][1], _MAX(v[1][1], v[2][1]));
	bboxMaxPt[2] = _MAX(v[0][2], _MAX(v[1][2], v[2][2]));

	return bboxMaxPt;
}


void Triangle::ComputeCenter()
{
	center = (v[0]+v[1]+v[2])/3.0f;
}


void Triangle::ComputeArea()
{
	Vector3d normal  = (v[1] - v[0]).cross(v[2] - v[0]);
	area  = 0.5f * normal.norm();
}

void Triangle::ComputeNormal()
{
	Vector3d tempNor = (v[1] - v[0]).cross(v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
	double tempNorLen = tempNor.norm();

	if ( tempNorLen != 0 )    normal = tempNor / tempNorLen;
	else                      normal = Vector3d(1,0,0);     // Note: this default vector also can be others
}

void Triangle::CorrectNormal(Vector3d tagtNormal)
{
	// Compute initial normal
	ComputeNormal();

	// Rearrange vertex order if needed
	double dotp = normal.dot(tagtNormal);
	if ( dotp < 0 )
	{
		Vector3d triVers[3];
		for (int i=0; i<3; i++)
		{
			triVers[i] = v[i];
		}

		v[0] = triVers[0];
		v[1] = triVers[2];
		v[2] = triVers[1];
	}

	// Recompute the normal
	ComputeNormal();
}

void Triangle::Draw()
{
	glBegin(GL_TRIANGLES);
	glNormal3f(normal[0], normal[1], normal[2]);
	glVertex3f(v[0][0], v[0][1], v[0][2]);
	glVertex3f(v[1][0], v[1][1], v[1][2]);
	glVertex3f(v[2][0], v[2][1], v[2][2]);
	glEnd();
}

void Triangle::DrawWire(double width, Vector3d color)
{
	glDisable(GL_LIGHTING);

	glLineWidth( width );
	glColor3f( color[0], color[1], color[2] );

	glBegin(GL_LINE_LOOP);
	glVertex3f( v[0][0], v[0][1], v[0][2] );
	glVertex3f( v[1][0], v[1][1], v[1][2] );
	glVertex3f( v[2][0], v[2][1], v[2][2] );
	glEnd();

	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}
