///////////////////////////////////////////////////////////////
//
// Utility/HelpFunc.h
//
//   Utility Tool Functions
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 03/Aug/2016
//
//
///////////////////////////////////////////////////////////////

#ifndef _HELP_FUNC_H
#define _HELP_FUNC_H

#include <vector>
#include "GeometricPrimitives.h"


using namespace std;


///////////////////////////////////////////////////////////////
// Define Variables
///////////////////////////////////////////////////////////////

// Default Part ID
#define ELEMENT_OUT_LIST            -1
#define VOXEL_OUT_LIST              -1
#define HEAP_SORT_Type           float


///////////////////////////////////////////////////////////////
// Function Declaration
///////////////////////////////////////////////////////////////

// Distance
float PointLineDistance(Vector3f tagtPt, Vector3f endPt1, Vector3f endPt2);
float PointPlaneDistance(Vector3f tagtPt, Vector3f point, Vector3f normal);
double PointSegmentDistance(Vector2f pt, Vector2f a, Vector2f b, Vector2f &c);

// Projection
Vector3f ProjectPointToLine(Vector3f tagtPt, Vector3f linePt, Vector3f lineDir);
Vector3f ProjectPointToPlane(Vector3f tagtPt, Plane plane);
Triangle* ProjectTriangleToPlane(Triangle *tri, Plane plane);

Vector3f ProjectPointOnLine_(Vector3f linePt0, Vector3f linePt1, Vector3f tagtPt);  // TODO: merge this function with ProjectPointToLine(..)

// Intersection
double LineHitTriangle(vec rayOrg, vec rayDir, Triangle triangle, vec &hitCoord, bool isPrint);
bool IsPointInsideTriangle(vec point, Triangle triangle, bool isPrint);
bool IsPointInsidePlane(Vector3f point, Vector3f planePoint, Vector3f planeNormal);
int TriTriIntersect(Triangle *meshTri, Triangle linkTri);
bool IsBoxBoxOverlap(Vector3f boxAMinPt, Vector3f boxAMaxPt, Vector3f boxBMinPt, Vector3f boxBMaxPt);

// Area
float GetTriangleArea(Vector3f triPt0, Vector3f triPt1, Vector3f triPt2);

// Sorting
vector<int> HeapSort(vector<HEAP_SORT_Type> &Array, bool isAscend);
void siftDown(vector<HEAP_SORT_Type> &Array, vector<int> &Indices, int start, int end, bool isAscend);
vector<int> BubbleSort(vector<float> &Array, bool isAscend);
vector<int> BubbleSort(vector<int> &Array, bool isAscend);

// Search within a List
int GetElementIndexInList(int targtElement, vector<int> elementList);
int GetPointIndexInList(Vector3f tagtPoint, vector<Vector3f> pointList);

// Matrix Calculation
void MultiplyPoint(vec inPt, double inMat[16], vec &outPt);
void MultiplyVector(vec inVec, double inMat[16], vec &outVec);
void MultiplyNormal(vec inNor, double inMat[16], vec &outNor);
void MultiplyMatrix(double inLefMat[16], double inRigMat[16], double outMat[16]);
void RotateMatrix(double angle, double x, double y, double z, double mat[16]);
void ScaleMatrix(double x, double y, double z, double mat[16]);
void TranslateMatrix(double x, double y, double z, double mat[16]);
void LoadIdentityMatrix(double mat[16]);
bool EqualMatrix(double matA[16], double matB[16]);
void PrintMatrix(double matrix[]);

void PrintVector(vector<Vector3f> points);

// Transform Matrix
void GetRotationMatrix(float rotAngle, Vector3f rotAxis, Vector3f rotCen, double rotMat[16]);
void GetPlaneTransformMatrix(Vector3f planeNormal, Vector3f planeVec, double transMat[16]);
void GetRotationMatrix(float rotAngle, Vector3f rotAxis, double rotMat[16]);
Vector3f RotateVector(Vector3f rotCenter, Vector3f rotAxis, float rotAngle, Vector3f tagtPt);

// String and Time
void GetFolderPath(const char filePath[], char folderPath[]);
void GetFileNameNoExtension(const char filePath[], char fileName[]);
string GetFileType(const char filePath[]);
string GetLocalTime();

#if USE_OPENGL_DRAW
// Draw Geometric Primitive
void DrawPoints(vector<Vector3f> points, float size, Vector3f color);
void DrawPolygon(vector<Vector3f> poly, float width, Vector3f color);
void DrawLine(Vector3f start_point, Vector3f end_point, float width, Vector3f color);
void DrawSolidCuboid(Vector3f minPt, Vector3f maxPt, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission);
void DrawWireCuboid(Vector3f minPt, Vector3f maxPt, float lineWidth, Vector3f color);
void DrawSphere(Vector3f position, float radius, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission);
void DrawWireSphere(Vector3f center, float radius, float lineWidth);
void DrawCylinder(Vector3f p1, Vector3f p2, float radius, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission); 
float Draw2DTextAt(char *str, float offsetX, float offsetY, float size, int alignment, int spaceShift, int pointSize);


void DrawPlane(Vector3f point, Vector3f normal, float radius, Vector3f color);
void Draw3DText(Vector3f texPosition, float textScale, float lineWidth, int textNumber);
vector<int> GetNumberDigits(int number);
void Draw3DDigit(int unit);
#endif

Vector3f ColorMapping(float min, float max, float value);
Vector3f GetColor(float min, float max, float value, Vector3f minColor, Vector3f maxColor);

#endif