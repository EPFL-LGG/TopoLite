///////////////////////////////////////////////////////////////
//
// PolyPolyTest.h
//
//   Intersection Test between Two Planar Convex Polygons, following the approach in:
//     https://www.swtestacademy.com/intersection-convex-polygons-algorithm/
//
// by Song Peng ( songpenghit@gmail.com )
//
// 21/July/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef _POLY_POLY_TEST_H
#define _POLY_POLY_TEST_H



#include "vec.h"
#include <vector>
#include "Mesh/Polygon.h"
using namespace std;


///////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////

// Compute overlap of two 2D polygons
bool IsConvexPolygonIntersec(vector<Vector2f> &polyA, vector<Vector2f> &polyB, double &area);
vector<Vector3f> ConvexPolygonIntersec(vector<Vector3f> polyA, vector<Vector3f> polyB);
vector<Vector3f> PolyPolyIntersect(vector<Vector3f> polyA, vector<Vector3f> polyB);
bool IsPointInPolygon(Vector3f point, vector<Vector3f> polygon);
bool IsEqual(double d1, double d2);
bool GetIntersectionPoint(Vector3f l1p1, Vector3f l1p2, Vector3f l2p1, Vector3f l2p2, Vector3f &intersectPt);
vector<Vector3f> GetIntersectionPoints(Vector3f l1p1, Vector3f l1p2, vector<Vector3f> polygon);

// Post-process the overlap polygon
vector<Vector3f> ProjectPolygonTo3D(vector<Vector3f> poly, double projMat[]);
void RearrangePoints(vector<Vector3f> &points);


#endif