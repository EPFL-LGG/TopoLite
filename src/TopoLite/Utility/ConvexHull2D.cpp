///////////////////////////////////////////////////////////////
//
// ConvexHull_2D.cpp
//
//   Compute 2D Convex Hull for a set of co-planar points
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 10/Nov/2017
//
//
///////////////////////////////////////////////////////////////

#include <algorithm>

//**************************************************************************************//
//                              Compute 2D Convex Hull
//**************************************************************************************//

// Compute 2D Convex Hull wiht Monotone Chain method:
// https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
// 
// Note: 
// 1) the z-coordinate of elements in input point list P is ignored;
// 2) Returns a list of points on the convex hull in counter-clockwise order,
//    where the last point in the returned list is the same as the first one.

template<typename Scalar>
void ConvexHull2D<Scalar>::compute(const ListVector3 &in, ListVector3 &out)
{
    out.clear();
	int n = in.size(), k = 0;
	if (n == 1){
	    return;
	}

	out.clear();
	out.resize(2 * n);

	// Sort points lexicographically
	ListVector3 _in = in;
	sortXY(_in);

	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && cross(out[k - 2], out[k - 1], _in[i]) <= 0) k--;
		out[k++] = _in[i];
	}

	// Build upper hull
	for (int i = n - 2, t = k + 1; i >= 0; i--) {
		while (k >= t && cross(out[k - 2], out[k - 1], _in[i]) <= 0) k--;
		out[k++] = _in[i];
	}

	out.resize(k - 1);

	reverse(out.begin(), out.end());
}

template<typename Scalar>
void ConvexHull2D<Scalar>::compute(const ListVector3 &in, Vector3 normal, ListVector3 &out){

    Vector3 center(0, 0, 0);
    for(Vector3 pt: in){
        center += pt;
    }
    center /= in.size();

    Vector3 x_axis, y_axis;
    x_axis = normal.cross(Vector3(1, 0, 0));
    if(x_axis.norm() < FLOAT_ERROR_LARGE)
        x_axis = normal.cross(Vector3(0, 1, 0));
    x_axis.normalize();

    y_axis = normal.cross(x_axis);
    y_axis.normalize();

    ListVector3 in2D;
    for(Vector3 pt: in){
        in2D.push_back(Vector3((pt - center).dot(x_axis), (pt - center).dot(y_axis), 0));
    }

    compute(in2D, out);

    for(Vector3 &pt: out){
        pt = pt[0] * x_axis + pt[1] * y_axis + center;
    }

    return;
}




// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
template<typename Scalar>
Scalar ConvexHull2D<Scalar>::cross(const Vector3 &O, const Vector3 &A, const Vector3 &B)
{
	return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}



//**************************************************************************************//
//                       Sort Points Based on X- and Y-coordinate
//**************************************************************************************//

template<typename Scalar>
void ConvexHull2D<Scalar>::sortXY(ListVector3 &Array)
{
	std::sort(Array.begin(), Array.end(), [=](const Vector3 &a, const Vector3 &b){
        if (a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()))
            return true;
        else
            return false;
	});
}
