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
	polyType = POLY_NONE;
}

template <typename Scalar>
_Polygon<Scalar>::~_Polygon()
{
    clear();
}

template <typename Scalar>
_Polygon<Scalar>::_Polygon(const _Polygon<Scalar> &poly)
{
    vers.clear();
    for(pVertex vertex: poly.vers){
        vers.push_back(make_shared<VPoint<Scalar>>(*vertex));
    }

    texs.clear();
    for(pVTex tex: poly.texs){
        texs.push_back(make_shared<VTex<Scalar>>(*tex));
    }

    this->polyType = poly.polyType;
}

//**************************************************************************************//
//                                    modify operation
//**************************************************************************************//

// 1) will affect computed data
template <typename Scalar>
void _Polygon<Scalar>::setVertices(vector<Vector3> _vers)
{

    vers.clear();
    for (size_t i = 0; i < _vers.size(); i++)
    {
        pVertex vertex = make_shared<VPoint<Scalar>>(_vers[i]);
        vers.push_back(vertex);
    }
}

template <typename Scalar>
size_t _Polygon<Scalar>::push_back(Vector3 pt)
{
    pVertex vertex = make_shared<VPoint<Scalar>>(pt);
    vers.push_back(vertex);
    return vers.size();
}

template <typename Scalar>
size_t _Polygon<Scalar>::push_back(Vector3 pt, Vector2 tex)
{
    pVertex vertex = make_shared<VPoint<Scalar>>(pt);
    vers.push_back(vertex);

    pVTex ptex = make_shared<VTex<Scalar>>(tex);
    texs.push_back(ptex);

    return vers.size();
}

// 2) will not affect computed data

template <typename Scalar>
void _Polygon<Scalar>::reverseVertices()
{
    // Reverse vertices
    vector<pVertex> newVers;
    for (int i = (int)(vers.size()) - 1; i >= 0; i--)
    {
        pVertex vertex = make_shared<VPoint<Scalar>>(*vers[i]);
        newVers.push_back(vertex);
    }

    vers = newVers;
}

template <typename Scalar>
void _Polygon<Scalar>::translatePolygon(Vector3 transVec)
{
    for (size_t i = 0; i < vers.size(); i++)
    {
        vers[i]->pos += transVec;
    }
}

template <typename Scalar>
void _Polygon<Scalar>::translatePolygonTex(Vector2 transVec)
{
    for (size_t i = 0; i < texs.size(); i++)
    {
        texs[i]->texCoord += transVec;
    }
}

/***********************************************
 *                                             *
 *             read only      operation        *
 *                                             *
 ***********************************************/

template <typename Scalar>
bool _Polygon<Scalar>::checkEquality(const _Polygon &poly) const
{
	if (this->size() != poly.size())
		return false;

	const _Polygon<Scalar> &A = *this;
	const _Polygon<Scalar> &B = poly;

	int id;
    for(id = 0; id < A.size(); id++)
    {
        if((A.vers[id]->pos - B.vers[0]->pos).norm() < FLOAT_ERROR_SMALL){
            break;
        }
    }
    if(id == A.size()) return false;

    for(size_t jd = 0; jd < A.size(); jd++)
    {
        int Aij = (id + jd) % A.size();
        if((A.vers[Aij]->pos - B.vers[jd]->pos).norm() > FLOAT_ERROR_SMALL){
            return false;
        }
    }

    return true;
}

template <typename Scalar>
void _Polygon<Scalar>::print() const
{
	printf("verNum: %lu \n", vers.size());
	for (size_t i = 0; i < vers.size(); i++)
	{
		printf("(%6.3f, %6.3f, %6.3f) \n", vers[i]->pos.x(), vers[i]->pos.y(), vers[i]->pos.z());
	}
	printf("\n");
}

template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::center() const
{
    Vector3 _center = Vector3(0, 0, 0);
	for (size_t i = 0; i < vers.size(); i++)
	{
        _center += vers[i]->pos;
	}
    _center = _center / vers.size();
	return _center;
}

template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::normal() const
{
	Vector3 _center = center();
	Vector3 tempNor(0, 0, 0);
	for(int id = 0; id < (int)(vers.size()) - 1; id++)
	{
	    tempNor += (vers[id]->pos - _center).cross(vers[id + 1]->pos - _center);
	}

	if(tempNor.norm() < FLOAT_ERROR_LARGE)
	    return Vector3(0, 0, 0);

	// normal already normalized
	Vector3 _normal = computeFitedPlaneNormal();
	if(_normal.dot(tempNor) < 0) _normal *= -1;
	return _normal;
}

// see https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::computeFitedPlaneNormal() const
{
	if(vers.size() < 3)
	{
		return Vector3(0, 0, 0);
	}

	Vector3 _center = center();

	double xx, xy, xz, yy, yz, zz;
	xx = xy = xz = yy = yz = zz = 0;
	for(pVertex ver: vers)
	{
		Vector3 r = ver->pos - _center;
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

	Vector3 _normal;
	if(maxDet == det_x)
	{
        _normal = Vector3(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
	}
	else if(maxDet == det_y)
	{
        _normal = Vector3(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
	}
	else {
        _normal = Vector3(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	}

	return _normal.normalized();
}

template <typename Scalar>
Scalar _Polygon<Scalar>::area() const
{
    Vector3 _normal = normal();

	Scalar signedArea = 0;
	for (size_t i = 0; i < vers.size(); i++)
	{
        Vector3 currVer = vers[i]->pos;
        Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
		signedArea += 0.5 * (_normal.dot(currVer.cross(nextVer)));
	}

	return std::abs( signedArea );
}

template <typename Scalar>
Scalar _Polygon<Scalar>::average_edge() const
{
	Scalar avgEdgeLen = 0;

	for (size_t i = 0; i < vers.size(); i++)
	{
		Vector3 currVer = vers[i]->pos;
		Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
        avgEdgeLen += (nextVer - currVer).norm();
	}

	avgEdgeLen /= vers.size();

	return avgEdgeLen;
}

template <typename Scalar>
Scalar _Polygon<Scalar>::max_radius() const
{
    Scalar MaxRadius = 0;

    Vector3 origin = center();
    for (size_t i = 0; i < vers.size(); i++)
    {
        MaxRadius = std::max((vers[i]->pos - origin).norm(), MaxRadius);
    }
    return MaxRadius;
}

template <typename Scalar>
void _Polygon<Scalar>::computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const
{
	Vector3 _normal = normal();
    origin = center();

	x_axis = _normal.cross(Vector3(1, 0, 0));
	if(x_axis.norm() < FLOAT_ERROR_LARGE)
	    x_axis = _normal.cross(Vector3(0, 1, 0));
	x_axis.normalize();

	y_axis = _normal.cross(x_axis);
	y_axis.normalize();
}

// https://www.mn.uio.no/math/english/people/aca/michaelf/papers/wach_mv.pdf
template <typename Scalar>
vector<Scalar> _Polygon<Scalar>::computeBaryCentric(Vector2 pt) const{
    vector<Scalar> barycentric;

    if(texs.empty()){
        return barycentric;
    }

    vector<Scalar> wi;
    Scalar sumw = 0;
    for(int id = 0; id < texs.size(); id++)
    {
        Vector2 mv = texs[(id - 1 + texs.size()) % texs.size()]->texCoord;
        Vector2 v =  texs[id]->texCoord;
        Vector2 pv = texs[(id + 1 + texs.size()) % texs.size()]->texCoord;

        Triangle<Scalar> A_mv_v_pv(mv, v, pv);
        Triangle<Scalar> A_pt_mv_v(pt, mv, v);
        Triangle<Scalar> A_pt_v_pv(pt, v, pv);

        Scalar area_pt_mv_v = A_pt_mv_v.computeSignedArea();
        Scalar area_pt_v_pv = A_pt_v_pv.computeSignedArea();

        // line case 1)
        if(std::abs(area_pt_mv_v) < FLOAT_ERROR_SMALL){
            vector<Scalar> line = computeBaryCentric(mv, v, pt);
            barycentric.resize(texs.size(), 0);
            barycentric[(id - 1 + texs.size()) % texs.size()] = line[0];
            barycentric[id] = line[1];
            return barycentric;
        }

        // line case 2)
        if(std::abs(area_pt_v_pv) < FLOAT_ERROR_SMALL){
            vector<Scalar> line = computeBaryCentric(v, pv, pt);
            barycentric.resize(texs.size(), 0);
            barycentric[id] = line[0];
            barycentric[(id + 1) % texs.size()] = line[1];
            return barycentric;
        }

        Scalar w = A_mv_v_pv.computeSignedArea() / area_pt_mv_v / area_pt_v_pv;
        wi.push_back(w);
        sumw += w;
    }

    for(size_t id = 0; id < texs.size(); id++){
        barycentric.push_back(wi[id] / sumw);
    }

    return barycentric;
}

template <typename Scalar>
vector<Scalar> _Polygon<Scalar>::computeBaryCentric(Vector2 sta, Vector2 end, Vector2 pt) const{

    // notice we don't handle the case where pt is not on the segment (sta, end)
    if((pt - sta).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>({1, 0});
    }

    if((pt - end).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>({0, 1});
    }

    if((sta - end).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>();
    }

    Scalar l0 = (pt - end).norm() / (sta - end).norm();
    return vector<Scalar>({l0, 1 - l0});
}



template <typename Scalar>
void _Polygon<Scalar>::convertToTriangles(vector<pTriangle> &tris) const
{
    if(vers.size() < 3)
        return;

    Vector3 _center = center();
    for (size_t i = 0; i < vers.size(); i++)
	{
		pTriangle tri = make_shared<Triangle<Scalar>>();
		tri->v[0] = _center;
		tri->v[1] = vers[i]->pos;
		tri->v[2] = vers[(i + 1)%vers.size()]->pos;
		tris.push_back(tri);
	}

    return;
}

template <typename Scalar>
int _Polygon<Scalar>::getPtVerID(_Polygon<Scalar>::Vector3 point) const
{
	for (size_t i = 0; i < vers.size(); i++)
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

template class _Polygon<double>;
template class _Polygon<float>;
