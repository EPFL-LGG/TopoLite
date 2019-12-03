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


#include "ConvexHull2D.h"


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

vector<Vector3f> convex_hull(vector<Vector3f> P)
{
	int n = P.size(), k = 0;
	if (n == 1) return P;
	vector<Vector3f> H(2 * n);

	// Sort points lexicographically
	BubbleSort(P, true);

	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	// Build upper hull
	for (int i = n - 2, t = k + 1; i >= 0; i--) {
		while (k >= t && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
		H[k++] = P[i];
	}

	H.resize(k - 1);

	reverse(H.begin(), H.end());

	return H;
}

// 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
// Returns a positive value, if OAB makes a counter-clockwise turn,
// negative for clockwise turn, and zero if the points are collinear.
double cross(const Vector3f &O, const Vector3f &A, const Vector3f &B)
{
	return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}




//**************************************************************************************//
//                       Sort Points Based on X- and Y-coordinate
//**************************************************************************************//

vector<int> BubbleSort(vector<Vector3f> &Array, bool isAscend)
{
	vector<int> Indices;
	for (int i = 0; i < Array.size(); i++)
		Indices.push_back(i);

	//printf("Before Sorting: ");
	//for (int i=0; i<Array.size(); i++)
	//	printf(" %d: %.2f ", Indices[i], Array[i]);
	//printf("\n");

	int i, j, flag = 1; // Set flag to 1 to start first pass
	float tempValue;    // Holding variable
	int tempIndex;      // Holding variable index 
	int num = Array.size();
	for (i = 1; (i <= num) && flag; i++)
	{
		flag = 0;
		for (j = 0; j < (num - 1); j++)
		{
			if ((isAscend == true && IS_LESS(Array[j+1], Array[j]) == true ) ||
				(isAscend == false && IS_LESS(Array[j + 1], Array[j]) == false))
			{
				swap(Array[j], Array[j+1]);
				swap(Indices[j], Indices[j+1]);

				flag = 1;
			}
		}
	}

	//printf("After Sorting:  ");
	//for (int i=0; i<Array.size(); i++)
	//	printf(" %d: %.2f ", Indices[i], Array[i]);
	//printf("\n");

	return Indices;
}


bool IS_LESS(Vector3f a, Vector3f b)
{
	if (a.x < b.x || (a.x == b.x && a.y < b.y))
		return true;
	else
		return false;
}
