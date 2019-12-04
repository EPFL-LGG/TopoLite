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

#ifndef CATCH2_UNITTEST

#include "Utility/HelpStruct.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"
#include "Polygon.h"

//**************************************************************************************//
//                                    Initialization
//**************************************************************************************//

_Polygon::_Polygon()
{
	normal = Vector3f(0, 0, 0);
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
	this->verIDs = poly.verIDs;
	this->texIDs = poly.texIDs;
	this->polyType = poly.polyType;
	this->dist = poly.dist;
	//this->area   = poly.area;

	return *this;
}

_Polygon::_Polygon(const _Polygon &poly)
{
    this->vers   = poly.vers;
    this->normal = poly.normal;
    this->center = poly.center;
    this->verIDs = poly.verIDs;
    this->texIDs = poly.texIDs;
    this->polyType = poly.polyType;
    this->dist = poly.dist;
}

bool _Polygon::IsEqual(_Polygon *poly)
{
	if (this->vers.size() != poly->vers.size())
		return false;

	// TODO: may need to add more strict conditions (e.g., equal vertex positions)
	float dist = len(this->center - poly->center);
	if (dist > FLOAT_ERROR_LARGE)
		return false;

	return true;
}

void _Polygon::Print()
{
	printf("verNum: %lu \n", vers.size());
	for (int i = 0; i < vers.size(); i++)
	{
		printf("(%6.3f, %6.3f, %6.3f) \n", vers[i].pos.x, vers[i].pos.y, vers[i].pos.z);
	}
	printf("\n");
}

void _Polygon::SetVertices(vector<Vector3f> _vers)
{
	vers.clear();
	for (int i = 0; i < _vers.size(); i++)
	{
		vers.push_back( _Vertex(_vers[i]) );
	}
}

size_t _Polygon::push_back(Vector3f pt)
{
    vers.push_back(_Vertex(pt));
    return vers.size();
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
//                                    Compute Properties
//**************************************************************************************//

Vector3f _Polygon::ComputeCenter()
{
	center = Vector3f(0,0,0);
	for (int i=0; i<vers.size(); i++)
	{
		center += vers[i].pos;
	}

	center = center / float(vers.size());

	return center;
}


Vector3f _Polygon::ComputeNormal()
{
	ComputeCenter();

	Vector3f tempNor = (vers[0].pos - center) CROSS (vers[1].pos - center);

	PlaneNormalFit();
	if((normal DOT tempNor) < 0) normal *= -1;

	return normal;
}

//see https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
Vector3f _Polygon::PlaneNormalFit()
{
	if(vers.size() < 3)
	{
		return Vector3f(0, 0, 0);
	}

	Vector3f centroid = ComputeCenter();

	double xx, xy, xz, yy, yz, zz;
	xx = xy = xz = yy = yz = zz = 0;
	for(_Vertex ver: vers)
	{
		Vector3f r = ver.pos - centroid;
		xx += r.x * r.x;
		xy += r.x * r.y;
		xz += r.x * r.z;
		yy += r.y * r.y;
		yz += r.y * r.z;
		zz += r.z * r.z;
	}

	double det_x = yy*zz - yz*yz;
	double det_y = xx*zz - xz*xz;
	double det_z = xx*yy - xy*xy;

	double maxDet = std::max(det_x, std::max(det_y, det_z));
	if(maxDet <= 0){
		return Vector3f(0, 0, 0);
	}

	if(maxDet == det_x)
	{
		normal = Vector3f(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
	}
	else if(maxDet == det_y)
	{
		normal = Vector3f(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
	}
	else {
		normal = Vector3f(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	};
	normal /= len(normal);
	return normal;
}

float _Polygon::ComputeArea()
{
	if ( normal == Vector3f(0,0,0) )
		ComputeNormal();

	float signedArea = 0;
	for (int i=0; i<vers.size(); i++)
	{
		Vector3f crossVec;

		if ( i < vers.size()-1 )    crossVec = vers[i].pos CROSS vers[i+1].pos;
		else                        crossVec = vers[i].pos CROSS vers[0].pos;

		signedArea += 0.5f * (normal DOT crossVec);
	}

	float area = fabs( signedArea );

	return area;
}

float _Polygon::ComputeAverageEdge()
{
	float avgEdgeLen = 0;

	for (int i = 0; i < vers.size(); i++)
	{
		Vector3f currVer = vers[i].pos;
		Vector3f nextVer = vers[(i + 1) % vers.size()].pos;
		float edgeLen = len(nextVer - currVer);

		avgEdgeLen += edgeLen;
	}

	avgEdgeLen /= (float)vers.size();

	return avgEdgeLen;
}

void _Polygon::ComputeFrame(Vector3f &x_axis, Vector3f &y_axis, Vector3f &origin)
{
	normal = ComputeNormal();
	center = ComputeCenter();
	x_axis = normal CROSS Vector3f(1, 0, 0);
	if(len(x_axis) < FLOAT_ERROR_SMALL) x_axis = normal CROSS Vector3f(0, 1, 0);
	x_axis /= len(x_axis);
	y_axis = normal CROSS x_axis; y_axis /= len(y_axis);
	origin = center;
}

vector<Vector3i> _Polygon::ProjectToNormalPlane(Vector3f x_axis, Vector3f y_axis, Vector3f origin, float Scale) {
	vector<Vector3i> pts;
	for(int id = 0; id < vers.size(); id++)
	{
		Vector3f pos = vers[id].pos;
		int x = (int)((pos - origin) DOT x_axis * Scale);
		int y = (int)((pos - origin) DOT y_axis * Scale);
		int z = 0;
		pts.push_back(Vector3i(x, y, z));
	}
	return pts;
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

void _Polygon::SetPolyType(float _polyType)
{
	polyType = _polyType;
}

int _Polygon::GetPolyType()
{
	return polyType;
}

vector<Vector3f> _Polygon::GetVertices()
{
	vector<Vector3f> vertices;

	for (int i = 0; i < vers.size(); i++)
	{
		vertices.push_back( vers[i].pos );
	}

	return vertices;
}


vector<Vector2f> _Polygon::GetVerticesTex() {
	vector<Vector2f> vertices;

	for (int i = 0; i < vers.size(); i++)
	{
		vertices.push_back( vers[i].texCoord );
	}
	return vertices;
}

//**************************************************************************************//
//                                  Polygon Operations
//**************************************************************************************//

void _Polygon::Convert2Triangles(vector<pTriangle> &tris)
{
    if(vers.size() < 3)
        return;

    center = ComputeCenter();
    for (int i = 0; i < (int)(vers.size()); i++)
	{
		pTriangle tri = make_shared<Triangle>();
		tri->v[0] = center;
		tri->v[1] = vers[i].pos;
		tri->v[2] = vers[(i + 1)%vers.size()].pos;
		tris.push_back(tri);
	}

    return;
}

int _Polygon::GetVertexIndexInList(Vector3f tagtVerPos)
{
	for (int i = 0; i < vers.size(); i++)
	{
		float dist = len(tagtVerPos - vers[i].pos);

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

void _Polygon::Translate(Vector3f transVec)
{
	for (int i = 0; i < vers.size(); i++)
	{
		vers[i].pos += transVec;
	}

	center += transVec;
}

vector<Vector3f> _Polygon::ProjectPolygonTo2D(double projMat[])
{
	vector<Vector3f> poly2D;

	for (int i = 0; i < vers.size(); i++)
	{
		Vector3f ver2D;
		MultiplyPoint( vers[i].pos, projMat, ver2D);

		poly2D.push_back( ver2D );
	}

	return poly2D;
}

void _Polygon::ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[])
{
	if ( vers.size() < 3 )
		return;

	Vector3f origin = ComputeCenter();

	Vector3f zAxis = ComputeNormal();
	Vector3f xAxis = vers[1].pos - vers[0].pos;
	xAxis = xAxis / len(xAxis);
	Vector3f yAxis = zAxis CROSS xAxis;

	double mat[16];
	identityd(mat);
	mat[0] = xAxis.x;     mat[1] = xAxis.y;     mat[2] = xAxis.z;
	mat[4] = yAxis.x;     mat[5] = yAxis.y;     mat[6] = yAxis.z;
	mat[8] = zAxis.x;     mat[9] = zAxis.y;     mat[10] = zAxis.z;
	mat[12] = origin.x;   mat[13] = origin.y;   mat[14] = origin.z;

	memcpy(invsProjMat, mat, sizeof(double)*16);

	memcpy(projMat, mat, sizeof(double)*16);
	if (invert4by4(projMat) == 0)  printf("Inverse Matrix Error \n");
}

#else

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include "Polygon.h"
#include <cmath>
TEST_CASE("Class _Polygon"){
    _Polygon a;
    a.push_back(Vector3f(1, 0, 0));
    a.push_back(Vector3f(0, 1, 0));
    a.push_back(Vector3f(0, 0, 1));
    REQUIRE(std::abs(a.ComputeArea() - std::sqrt(3.0)/2) < 1e-5);
}

#endif