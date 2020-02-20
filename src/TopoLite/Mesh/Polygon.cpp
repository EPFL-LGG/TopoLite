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

#include "Utility/GeometricPrimitives.h"
#include "Utility/HelpDefine.h"
#include "Polygon.h"

//**************************************************************************************//
//                                    Initialization
//**************************************************************************************//

template <typename Scalar>
_Polygon<Scalar>::_Polygon()
{
	normal = Vector3(0, 0, 0);
	dist = polyType = 0;
}

template <typename Scalar>
_Polygon<Scalar>::~_Polygon()
{
    clear();
}

template <typename Scalar>
_Polygon<Scalar> & _Polygon<Scalar>::operator=(const _Polygon<Scalar> &poly)
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

template <typename Scalar>
_Polygon<Scalar>::_Polygon(const _Polygon<Scalar> &poly)
{
    this->vers   = poly.vers;
    this->normal = poly.normal;
    this->center = poly.center;
    this->polyType = poly.polyType;
    this->dist = poly.dist;
}

template <typename Scalar>
bool _Polygon<Scalar>::checkEquality(const _Polygon &poly)
{
	if (this->size() != poly.size())
		return false;

	const _Polygon<Scalar> &A = *this;
	const _Polygon<Scalar> &B = poly;

	int id;
    for(id = 0; id < A.size(); id++)
    {
        if((A[id] - B[0]).norm() < FLOAT_ERROR_SMALL){
            break;
        }
    }
    if(id == A.size()) return false;

    for(int jd = 0; jd < A.size(); jd++){
        if((A[(id + jd) % A.size()] - B[jd]).norm() > FLOAT_ERROR_SMALL){
            return false;
        }
    }

    return true;
}

template <typename Scalar>
void _Polygon<Scalar>::print()
{
	printf("verNum: %lu \n", vers.size());
	for (int i = 0; i < vers.size(); i++)
	{
		printf("(%6.3f, %6.3f, %6.3f) \n", vers[i]->pos.x(), vers[i]->pos.y(), vers[i]->pos.z());
	}
	printf("\n");
}

template <typename Scalar>
void _Polygon<Scalar>::setVertices(vector<Vector3> _vers)
{
	vers.clear();
	for (int i = 0; i < _vers.size(); i++)
	{
	    pVertex vertex = make_shared<_Vertex<Scalar>>(_vers[i]);
		vers.push_back(vertex);
	}
}

template <typename Scalar>
size_t _Polygon<Scalar>::push_back(Vector3 pt)
{
    pVertex vertex = make_shared<_Vertex<Scalar>>(pt);
    vers.push_back(vertex);
    return vers.size();
}

template <typename Scalar>
void _Polygon<Scalar>::reverseVertices()
{
    // Reverse vertices
    vector<pVertex> newVers;
    for (int i = vers.size() - 1; i >= 0; i--)
    {
        pVertex vertex = make_shared<_Vertex<Scalar>>(*vers[i]);
        newVers.push_back(vertex);
    }

    vers = newVers;

    // Reverse normal
    normal = -normal;
}


//**************************************************************************************//
//                                    Compute Properties
//**************************************************************************************//

template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::computeCenter()
{
	center = Vector3(0, 0, 0);
	for (int i= 0; i < vers.size(); i++)
	{
		center += vers[i]->pos;
	}

	center = center / vers.size();

	return center;
}


template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::computeNormal()
{
	computeCenter();
	Vector3 tempNor(0, 0, 0);
	for(int id = 0; id < vers.size() - 1; id++)
	{
	    tempNor += (vers[id]->pos - center).cross(vers[id + 1]->pos - center);
	}

	if(tempNor.norm() < FLOAT_ERROR_LARGE)
	    return Vector3(0, 0, 0);

	//normal already normalized
	computeFitedPlaneNormal();
	
	if(normal.dot(tempNor) < 0) normal *= -1;
	return normal;
}

//see https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::computeFitedPlaneNormal()
{
	if(vers.size() < 3)
	{
		return Vector3(0, 0, 0);
	}

	Vector3 centroid = computeCenter();

	double xx, xy, xz, yy, yz, zz;
	xx = xy = xz = yy = yz = zz = 0;
	for(pVertex ver: vers)
	{
		Vector3 r = ver->pos - centroid;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}

	double det_x = yy*zz - yz*yz;
	double det_y = xx*zz - xz*xz;
	double det_z = xx*yy - xy*xy;

	double maxDet = std::max(det_x, std::max(det_y, det_z));
	if(maxDet <= 0){
		return Vector3(0, 0, 0);
	}

	if(maxDet == det_x)
	{
		normal = Vector3(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
	}
	else if(maxDet == det_y)
	{
		normal = Vector3(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
	}
	else {
		normal = Vector3(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	};

	normal.normalize();
	return normal;
}

template <typename Scalar>
Scalar _Polygon<Scalar>::computeArea()
{
	if ( normal == Vector3(0,0,0))
	    computeNormal();

	Scalar signedArea = 0;
	for (int i = 0; i < vers.size(); i++)
	{
        Vector3 currVer = vers[i]->pos;
        Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
		signedArea += 0.5 * (normal.dot(currVer.cross(nextVer)));
	}

	return std::abs( signedArea );
}

template <typename Scalar>
Scalar _Polygon<Scalar>::computeAverageEdge()
{
	Scalar avgEdgeLen = 0;

	for (int i = 0; i < vers.size(); i++)
	{
		Vector3 currVer = vers[i]->pos;
		Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
        avgEdgeLen += (nextVer - currVer).norm();
	}

	avgEdgeLen /= vers.size();

	return avgEdgeLen;
}

template <typename Scalar>
Scalar _Polygon<Scalar>::computeMaxRadius()
{
    Scalar MaxRadius = 0;

    Vector3 origin = computeCenter();
    for (int i = 0; i < vers.size(); i++)
    {
        MaxRadius = std::max((vers[i]->pos - origin).norm(), MaxRadius);
    }
    return MaxRadius;
}

template <typename Scalar>
void _Polygon<Scalar>::computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin)
{
	normal = computeNormal();
    origin = computeCenter();

	x_axis = normal.cross(Vector3(1, 0, 0));
	if(x_axis.norm() < FLOAT_ERROR_LARGE)
	    x_axis = normal.cross(Vector3(0, 1, 0));
	x_axis.normalize();

	y_axis = normal.cross(x_axis);
	y_axis.normalize();
}


//**************************************************************************************//
//                                  Polygon Operations
//**************************************************************************************//

template <typename Scalar>
void _Polygon<Scalar>::convertToTriangles(vector<pTriangle> &tris)
{
    if(vers.size() < 3)
        return;

    center = computeCenter();
    for (int i = 0; i < (int)(vers.size()); i++)
	{
		pTriangle tri = make_shared<Triangle<Scalar>>();
		tri->v[0] = center;
		tri->v[1] = vers[i]->pos;
		tri->v[2] = vers[(i + 1)%vers.size()]->pos;
		tris.push_back(tri);
	}

    return;
}

template <typename Scalar>
int _Polygon<Scalar>::getPtVerID(_Polygon<Scalar>::Vector3 point)
{
	for (int i = 0; i < vers.size(); i++)
	{
		Scalar dist = (point - vers[i]->pos).norm();

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

template <typename Scalar>
void _Polygon<Scalar>::executeTranslation(Vector3 transVec)
{
	for (int i = 0; i < vers.size(); i++)
	{
		vers[i]->pos += transVec;
	}

	center += transVec;
}

//template <typename Scalar>
//vector<Matrix<Scalar, 3, 1>> _Polygon<Scalar>::ProjectPolygonTo2D(double projMat[])
//{
//	vector<Vector3> poly2D;
//
//	for (int i = 0; i < vers.size(); i++)
//	{
//		Vector3 ver2D;
//		MultiplyPoint( vers[i]->pos, projMat, ver2D);
//
//		poly2D.push_back( ver2D );
//	}
//
//	return poly2D;
//}

//template <typename Scalar>
//vector<Eigen::Vector3i> _Polygon<Scalar>::ProjectToNormalPlane(Vector3 x_axis, Vector3 y_axis, Vector3 origin, float Scale)
//{
//	vector<Vector3i> pts;
//	for(int id = 0; id < vers.size(); id++)
//	{
//		Vector3 pos = vers[id]->pos;
//		int x = (int)((pos - origin).dot(x_axis) * Scale);
//		int y = (int)((pos - origin).dot(y_axis) * Scale);
//		int z = 0;
//		pts.push_back(Vector3i(x, y, z));
//	}
//	return pts;
//}
//
//template <typename Scalar>
//void _Polygon<Scalar>::ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[])
//{
//	if ( vers.size() < 3 )
//		return;
//
//	Vector3 origin = ComputeCenter();
//
//	Vector3 zAxis = ComputeNormal();
//	Vector3 xAxis = vers[1]->pos - vers[0]->pos;
//	xAxis = xAxis / len(xAxis);
//	Vector3 yAxis = zAxis.cross(xAxis);
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
// 	if (invert4by4(projMat) == 0)  printf("Inverse Matrix Error \n");
//}

//
