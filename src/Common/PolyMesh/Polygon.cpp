///////////////////////////////////////////////////////////////
//
// Polygon.cpp
//
//   3D Polygon (vertices may or may not be co-planar)
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifdef WIN32
#include <GL/glut.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

#include "Polygon.h"



//**************************************************************************************//
//                                    Initialization
//**************************************************************************************//

_Polygon::_Polygon()
{

}

_Polygon::~_Polygon()
{

}

_Polygon & _Polygon::operator=(const _Polygon &poly)
{
	if( this == &poly )
		return *this;

	this->vers   = poly.vers;
	this->normal = poly.normal;
	this->center = poly.center;
	//this->area   = poly.area;

	return *this;
}

bool _Polygon::IsEqual(_Polygon *poly)
{
	if (this->vers.size() != poly->vers.size())
		return false;

	// TODO: may need to add more strict conditions (e.g., equal vertex positions)
	double dist = (this->center - poly->center).norm();
	if (dist > FLOAT_ERROR_LARGE)
		return false;

	return true;
}

void _Polygon::Print()
{
	//printf("verNum %d: [ ", verIDs.size());

	//for (int i = 0; i < verIDs.size(); i++)
	//{
	//	printf(" %d ", verIDs[i]);
	//}
	//printf("]\n");

//	printf("verNum: %lu \n", vers.size());
//	for (int i = 0; i < vers.size(); i++)
//	{
//		printf("(%6.3f, %6.3f, %6.3f) \n", vers[i].pos.x, vers[i].pos.y, vers[i].pos.z);
//	}
//	printf("\n");
}

void _Polygon::SetVertices(vector<Vector3d> _vers)
{
	vers.clear();
	for (int i = 0; i < _vers.size(); i++)
	{
		vers.push_back( _Vertex(_vers[i]) );
	}
}




//**************************************************************************************//
//                                    Compute Properties
//**************************************************************************************//

Vector3d _Polygon::ComputeCenter()
{
	center = Vector3d(0,0,0);
	for (int i=0; i<vers.size(); i++)
	{
		center += vers[i].pos;
	}

	center = center / double(vers.size());

	return center;
}


Vector3d _Polygon::ComputeNormal()
{
	ComputeCenter();

	Vector3d tempNor = (vers[0].pos - center).cross(vers[1].pos - center);

	normal = tempNor / (tempNor).norm();

	return normal;
}

double _Polygon::ComputeArea()
{
	if ( normal == Vector3d(0,0,0) )
		ComputeNormal();

	double signedArea = 0;
	for (int i=0; i<vers.size(); i++)
	{
		Vector3d crossVec;

		if ( i < vers.size()-1 )    crossVec = vers[i].pos.cross(vers[i+1].pos);
		else                        crossVec = vers[i].pos.cross(vers[0].pos);

		signedArea += 0.5f * (normal.dot(crossVec));
	}

	double area = fabs( signedArea );

	return area;
}

double _Polygon::ComputeAverageEdge()
{
	double avgEdgeLen = 0;

	for (int i = 0; i < vers.size(); i++)
	{
		Vector3d currVer = vers[i].pos;
		Vector3d nextVer = vers[(i + 1) % vers.size()].pos;
		double edgeLen = (nextVer - currVer).norm();

		avgEdgeLen += edgeLen;
	}

	avgEdgeLen /= (double)vers.size();

	return avgEdgeLen;
}

void _Polygon::ReverseVertices()
{
	// Reverse vertices
	vector<_Vertex> newVers;
	for (int i = vers.size() - 1; i >= 0; i--)
	{
		newVers.push_back(vers[i]);
	}

	vers = newVers;

	// Reverse normal
	normal = -1.0f * normal;
}




//**************************************************************************************//
//                                  Polygon Operations
//**************************************************************************************//

void _Polygon::SetDistance(int _dist)
{
	dist = _dist;
}

int _Polygon::GetDistance()
{
	return dist;
}

void _Polygon::SetPolyType(double _polyType)
{
	polyType = _polyType;
}

int _Polygon::GetPolyType()
{
	return polyType;
}

vector<Vector3d> _Polygon::GetVertices()
{
	vector<Vector3d> vertices;

	for (int i = 0; i < vers.size(); i++)
	{
		vertices.push_back( vers[i].pos );
	}

	return vertices;
}




//**************************************************************************************//
//                                  Polygon Operations
//**************************************************************************************//

vector<Triangle*> _Polygon::Convert2Triangles()
{
	vector<Triangle*> tris;

	for (int i = 1; i < vers.size() - 1; i++)
	{
		Triangle *tri = new Triangle();

		tri->v[0] = vers[0].pos;
		tri->v[1] = vers[i].pos;
		tri->v[2] = vers[i + 1].pos;

		tris.push_back(tri);
	}

	return tris;
}

int _Polygon::GetVertexIndexInList(Vector3d tagtVerPos)
{
	for (int i = 0; i < vers.size(); i++)
	{
		double dist = (tagtVerPos - vers[i].pos).norm();

		// Note: this threshold depends on the scale of elements
		if (dist < FLOAT_ERROR_LARGE)
		{
			return i;
		}
	}
	return ELEMENT_OUT_LIST;
}

//**************************************************************************************//
//                                  Polygon Transforms
//**************************************************************************************//

void _Polygon::Translate(Vector3d transVec)
{
	for (int i = 0; i < vers.size(); i++)
	{
		vers[i].pos += transVec;
	}

	center += transVec;
}

vector<Vector3d> _Polygon::ProjectPolygonTo2D(double projMat[])
{
	vector<Vector3d> poly2D;

//	for (int i = 0; i < vers.size(); i++)
//	{
//		Vector3d ver2D;
//		MultiplyPoint( vers[i].pos, projMat, ver2D);
//
//		poly2D.push_back( ver2D );
//	}

	return poly2D;
}

void _Polygon::ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[])
{
//	if ( vers.size() < 3 )
//		return;
//
//	Vector3d origin = ComputeCenter();
//
//	Vector3d zAxis = ComputeNormal();
//	Vector3d xAxis = vers[1].pos - vers[0].pos;
//	xAxis = xAxis / len(xAxis);
//	Vector3d yAxis = zAxis CROSS xAxis;
//
//	double mat[16];
//	identityd(mat);
//	mat[0] = xAxis.x;     mat[1] = xAxis.y;     mat[2] = xAxis.z;
//	mat[4] = yAxis.x;     mat[5] = yAxis.y;     mat[6] = yAxis.z;
//	mat[8] = zAxis.x;     mat[9] = zAxis.y;     mat[10] = zAxis.z;
//	mat[12] = origin.x;   mat[13] = origin.y;   mat[14] = origin.z;
//
//	memcpy(invsProjMat, mat, sizeof(double)*16);
//
//	memcpy(projMat, mat, sizeof(double)*16);
//	if (invert4by4(projMat) == 0)  printf("Inverse Matrix Error \n");
}




//**************************************************************************************//
//                                      Rendering
//**************************************************************************************//

void _Polygon::DrawPolygon(double width, Vector3d color)
{
	glDisable(GL_LIGHTING);
	glPointSize(12.0);
	glLineWidth(width);

	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < vers.size(); i++)
	{
		glVertex3f(vers[i].pos[0], vers[i].pos[1], vers[i].pos[2]);
	}
	glEnd();

	glPointSize(1.0);
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}

void _Polygon::ComputeFrame(Vector3d &x_axis, Vector3d &y_axis, Vector3d &origin)
{
	normal = ComputeNormal();
	center = ComputeCenter();
	x_axis = normal.cross(Vector3d(1, 0, 0));
	if(x_axis.norm() < FLOAT_ERROR_SMALL) x_axis = normal.cross(Vector3d(0, 1, 0));
	x_axis /= x_axis.norm();
	y_axis =  normal.cross(x_axis);
	y_axis /= y_axis.norm();
	origin = center;
}

vector<Vector3i> _Polygon::ProjectToNormalPlane(Vector3d x_axis, Vector3d y_axis, Vector3d origin, double Scale) {
	vector<Vector3i> pts;
	for(int id = 0; id < vers.size(); id++)
	{
		Vector3d pos = vers[id].pos;
		int x = (int)((pos - origin).dot(x_axis * Scale));
		int y = (int)((pos - origin).dot(y_axis * Scale));
		int z = 0;
		pts.push_back(Vector3i(x, y, z));
	}
	return pts;
}

