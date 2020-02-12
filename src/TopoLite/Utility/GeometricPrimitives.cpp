///////////////////////////////////////////////////////////////
//
// GeometricPrimitives.cpp
//
//   Common Structures
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
//
///////////////////////////////////////////////////////////////

#include "GeometricPrimitives.h"
#include "HelpDefine.h"
#include <cmath>

//**************************************************************************************//
//                                     3D Box
//**************************************************************************************//




//**************************************************************************************//
//                                    3D Triangle
//**************************************************************************************//

template<typename Scalar>
void Triangle<Scalar>::Init(Vector3 _v0, Vector3 _v1, Vector3 _v2)
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

	this->normal = tri.normal;
	this->center = tri.center;
	this->area   = tri.area;

	return *this;
}

template<typename Scalar>
bool Triangle<Scalar>::IsEqual(Triangle *tri)
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

template<typename Scalar>
void Triangle<Scalar>::PrintTriangle()
{
	printf("v0: [%.12f %.12f %.12f] \n", v[0].x, v[0].y, v[0].z);
	printf("v1: [%.12f %.12f %.12f] \n", v[1].x, v[1].y, v[1].z);
	printf("v2: [%.12f %.12f %.12f] \n", v[2].x, v[2].y, v[2].z);
	printf("\n");
}

template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::GetBBoxMinPt()
{
	Vector3 bboxMinPt;

	bboxMinPt.x = _MIN(v[0].x, _MIN(v[1].x, v[2].x));
	bboxMinPt.y = _MIN(v[0].y, _MIN(v[1].y, v[2].y));
	bboxMinPt.z = _MIN(v[0].z, _MIN(v[1].z, v[2].z));

	return bboxMinPt;
}
template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::GetBBoxMaxPt()
{
	Vector3 bboxMaxPt;

	bboxMaxPt.x = _MAX(v[0].x, _MAX(v[1].x, v[2].x));
	bboxMaxPt.y = _MAX(v[0].y, _MAX(v[1].y, v[2].y));
	bboxMaxPt.z = _MAX(v[0].z, _MAX(v[1].z, v[2].z));

	return bboxMaxPt;
}

template<typename Scalar>
void Triangle<Scalar>::ComputeCenter()
{
	center = (v[0]+v[1]+v[2])/3.0f;
}

template<typename Scalar>
void Triangle<Scalar>::ComputeArea()
{
	Vector3 normal  = (v[1] - v[0]).cross(v[2] - v[0]);
	area  = 0.5f * len(normal);
}

template<typename Scalar>
void Triangle<Scalar>::ComputeNormal()
{
	Vector3 tempNor = (v[1] - v[0]).cross(v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
	float tempNorLen = len(tempNor);

	if ( tempNorLen != 0 )    normal = tempNor / tempNorLen;
	else                      normal = Vector3(1,0,0);     // Note: this default vector also can be others
}

template<typename Scalar>
void Triangle<Scalar>::CorrectNormal(Vector3 tagtNormal)
{
	// Compute initial normal
	ComputeNormal();

	// Rearrange vertex order if needed
	float dotp = normal.dot(tagtNormal);
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

	// Recompute the normal
	ComputeNormal();
}