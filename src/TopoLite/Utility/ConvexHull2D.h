///////////////////////////////////////////////////////////////
//
// ConvexHull_2D.h
//
//   Compute 2D Convex Hull for a set of co-planar points
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 10/Nov/2017
//
//
///////////////////////////////////////////////////////////////

#include "vec.h"
#include <vector>

using namespace std;

// Chain Hull Algorithm
vector<Vector3f> convex_hull(vector<Vector3f> P);
double cross(const Vector3f &O, const Vector3f &A, const Vector3f &B);

// Sort Points Based on X - and Y - coordinate
vector<int> BubbleSort(vector<Vector3f> &Array, bool isAscend);
bool IS_LESS(Vector3f a, Vector3f b);