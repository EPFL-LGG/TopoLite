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

float Plane::PointPlaneDistance(Vector3f tagtPt)
{
	Vector3f tagtvec = tagtPt - point;

	float dotP = tagtvec DOT normal;
	float dist = fabs( dotP );

	return dist;
}


int Plane::PointPlaneIntersect(Vector3f tagtPt, bool isPrint)
{
	Vector3f vec = tagtPt - point;
	float dotP = normal DOT vec;

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

int Plane::LineIntersectPoint(Line line, Vector3f &crossPt)
{
	// Check if the line intersects the plane
	int state = LinePlaneIntersect( line );
	if ( state != LINE_PLANE_INTERSECT )
	{
		//printf("Warning: Line does not intersect with the plane. \n");
		return state;
	}

	// Compute the intersected point between the line and the plane
	Vector3f rayOrg = line.point1;
	Vector3f rayDir = (line.point2-line.point1) / len(line.point2-line.point1);
	Vector3f tempVec = point - rayOrg;
	float m = (normal DOT tempVec) / (normal DOT rayDir);

	crossPt[0] = rayOrg[0] + m * rayDir[0];
	crossPt[1] = rayOrg[1] + m * rayDir[1];
	crossPt[2] = rayOrg[2] + m * rayDir[2];

	return state;
}

#if USE_OPENGL_DRAW
// TODO: refine this function and remove duplicated ones
void Plane::DrawFace(float radius, Vector3f color)
{
	Vector3f linePt = point + 0.3f * normal; 

	//Vector3f localX = normal CROSS Vector3f(0,0,1); 
	Vector3f localX = normal CROSS Vector3f(0.5,0.5,0.5); 
	Vector3f localY = normal CROSS localX; 

	normal = normal / len(normal);
	localX = localX / len(localX);
	localY = localY / len(localY);

	//printf("N: [%.2f %.2f %.2f] \n", normal.x, normal.y, normal.z);
	//printf("X: [%.2f %.2f %.2f] \n", localX.x, localX.y, localX.z);
	//printf("Y: [%.2f %.2f %.2f] \n", localY.x, localY.y, localY.z);

	glDisable(GL_LIGHTING);
	glPointSize(12.0);
	glLineWidth(3.0);

	// Draw the plane point
	glColor3f(0.9,0.6,0.3);
	glBegin(GL_POINTS);
	glVertex3f(point.x, point.y, point.z);
	glEnd();

	// Draw the plane normal
	glColor3f(0.9,0.6,0.3);
	glBegin(GL_LINES);
	glVertex3f(point.x,  point.y,  point.z);
	glVertex3f(linePt.x, linePt.y, linePt.z);
	glEnd();

	// Draw the plane disk
	glColor3f(color.x, color.y, color.z);
	glBegin(GL_POLYGON);
	int polyPtNum = 20;
	for (int i=0; i<polyPtNum; i++)
	{
		float theta = i* 2*M_PI / (float)polyPtNum;
		Vector3f polyPt = point + radius*(localX*cos(theta) + localY*sin(theta));
		glVertex3f(polyPt.x, polyPt.y, polyPt.z);
	}
	glEnd();

	glEnable(GL_LIGHTING);
	glPointSize(1.0);
	glLineWidth(1.0);
}
#endif


//**************************************************************************************//
//                                     3D Box
//**************************************************************************************//

Box::Box()
{
	minPt = Vector3f(0, 0, 0);
	maxPt = Vector3f(0, 0, 0);
	cenPt = Vector3f(0, 0, 0);
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
	printf("Box: [%7.3f  %7.3f  %7.3f]      [%7.3f  %7.3f  %7.3f] \n", minPt.x, minPt.y, minPt.z, maxPt.x, maxPt.y, maxPt.z);
}


void Box::GetCenter()
{
	cenPt = 0.5f * ( minPt + maxPt );
}

void Box::GetSize()
{
	size = maxPt - minPt;
}

void Box::Transform(Vector3f transVec, Vector3f scale)
{
	minPt.x *= scale.x;  minPt.y *= scale.y;  minPt.z *= scale.z;
	maxPt.x *= scale.x;  maxPt.y *= scale.y;  maxPt.z *= scale.z;
	cenPt.x *= scale.x;  cenPt.y *= scale.y;  cenPt.z *= scale.z;

	minPt += transVec;
	maxPt += transVec;
	cenPt += transVec;
}

float Box::GetQuadArea()
{
	Vector3f dimen = maxPt - minPt;
	float quadArea = 0;

	if      ( dimen.x == 0 && dimen.y  > 0 &&  dimen.z  > 0 )    quadArea = dimen.y * dimen.z;  // y-z plane quad
	else if ( dimen.x  > 0 && dimen.y == 0 &&  dimen.z  > 0 )    quadArea = dimen.x * dimen.z;  // x-z plane quad
	else if ( dimen.x  > 0 && dimen.y  > 0 &&  dimen.z == 0 )    quadArea = dimen.x * dimen.y;  // x-y plane quad
	else                                                         printf("Warning: The box is not degenerated into a quad. \n");
		
	return quadArea;
}




//**************************************************************************************//
//                                    3D Triangle
//**************************************************************************************//

void Triangle::Init(Vector3f _v0, Vector3f _v1, Vector3f _v2)
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
	printf("v0: [%.12f %.12f %.12f] \n", v[0].x, v[0].y, v[0].z);
	printf("v1: [%.12f %.12f %.12f] \n", v[1].x, v[1].y, v[1].z);
	printf("v2: [%.12f %.12f %.12f] \n", v[2].x, v[2].y, v[2].z);
	printf("\n");
}

Vector3f Triangle::GetBBoxMinPt()
{
	Vector3f bboxMinPt;

	bboxMinPt.x = _MIN(v[0].x, _MIN(v[1].x, v[2].x));
	bboxMinPt.y = _MIN(v[0].y, _MIN(v[1].y, v[2].y));
	bboxMinPt.z = _MIN(v[0].z, _MIN(v[1].z, v[2].z));

	return bboxMinPt;
}

Vector3f Triangle::GetBBoxMaxPt()
{
	Vector3f bboxMaxPt;

	bboxMaxPt.x = _MAX(v[0].x, _MAX(v[1].x, v[2].x));
	bboxMaxPt.y = _MAX(v[0].y, _MAX(v[1].y, v[2].y));
	bboxMaxPt.z = _MAX(v[0].z, _MAX(v[1].z, v[2].z));

	return bboxMaxPt;
}


void Triangle::ComputeCenter()
{
	center = (v[0]+v[1]+v[2])/3.0f;
}


void Triangle::ComputeArea()
{
	Vector3f normal  = (v[1] - v[0]) CROSS (v[2] - v[0]);
	area  = 0.5f * len(normal);
}

void Triangle::ComputeNormal()
{
	Vector3f tempNor = (v[1] - v[0]) CROSS (v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
	float tempNorLen = len(tempNor);

	if ( tempNorLen != 0 )    normal = tempNor / tempNorLen;
	else                      normal = Vector3f(1,0,0);     // Note: this default vector also can be others
}

void Triangle::CorrectNormal(Vector3f tagtNormal)
{
	// Compute initial normal
	ComputeNormal();

	// Rearrange vertex order if needed
	float dotp = normal DOT tagtNormal;
	if ( dotp < 0 )
	{
		Vector3f triVers[3];
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

#if USE_OPENGL_DRAW
void Triangle::Draw()
{
	glBegin(GL_TRIANGLES);
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(v[0].x, v[0].y, v[0].z);
	glVertex3f(v[1].x, v[1].y, v[1].z);
	glVertex3f(v[2].x, v[2].y, v[2].z);
	glEnd();
}

void Triangle::DrawWire(float width, Vector3f color)
{
	glDisable(GL_LIGHTING);

	glLineWidth( width );
	glColor3f( color.x, color.y, color.z );

	glBegin(GL_LINE_LOOP);
	glVertex3f( v[0].x, v[0].y, v[0].z );
	glVertex3f( v[1].x, v[1].y, v[1].z );
	glVertex3f( v[2].x, v[2].y, v[2].z );
	glEnd();

	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}
#endif