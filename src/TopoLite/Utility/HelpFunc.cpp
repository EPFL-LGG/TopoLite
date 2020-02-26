/////////////////////////////////////////////////////////////////
////
//// HelpFunc.cpp
////
////   Utility Tool Functions
////
//// by Peng SONG ( songpenghit@gmail.com )
////
//// 03/Aug/2016
////
/////////////////////////////////////////////////////////////////
//
//
//#include "math3D.h"
//#include "HelpDefine.h"
//#include "HelpFunc.h"
//#include <ctime>
//#include <Eigen/Dense>
//
//// Render 3D Primitive
//#if USE_OPENGL_DRAW
//GLUquadricObj *quadObj = gluNewQuadric();
//#endif
//
//// Color scheme table
//Vector3f colorTable[18] = {
//	Vector3f(0.9, 0.4, 0.4),   //  1: Red
//	Vector3f(0.4, 0.4, 0.9),   //  2: Blue
//	Vector3f(0.9, 0.9, 0.5),   //  3: Yellow
//	Vector3f(0.9, 0.5, 0.3),   //  4: Orange
//	Vector3f(0.7, 0.3, 0.9),   //  5: Purple
//	Vector3f(0.3, 0.9, 0.9),   //  6: Cyan
//	Vector3f(0.6, 0.4, 0.3),   //  7: Brown
//	Vector3f(0.9, 0.3, 0.6),   //  8: Pink
//	Vector3f(0.4, 0.6, 0.7),   //  9: Gray -> Dark Cyan
//	Vector3f(0.9, 0.6, 0.5),   // 10: LightSalmon
//	Vector3f(0.3, 0.9, 0.3),   // 11: Green
//	Vector3f(0.5, 0.2, 0.5),   // 12: Dark Purple
//	Vector3f(0.4, 0.8, 0.7),   // 13: Dark Cyan
//	Vector3f(0.3, 0.4, 0.7),   // 14: Dark Blue
//	Vector3f(0.6, 0.6, 0.3),    // 15: Dark Yellow
//	Vector3f(0.77, 0.64, 0.45),   //  16: Orange
//	Vector3f(0.54, 0.71, 0.80),   //  17: Cyan
//	Vector3f(0.54, 0.71, 0.20),   //  18: Cyan
//};
//
//
//
//
////**************************************************************************************//
////                                    Distance
////**************************************************************************************//
//
//float PointLineDistance(Vector3f tagtPt, Vector3f endPt1, Vector3f endPt2)
//{
//	Vector3f tagtvec = tagtPt - endPt1;
//	Vector3f lineVec = (endPt2-endPt1) / len(endPt2-endPt1);
//
//	Vector3f crossP = tagtvec CROSS lineVec;
//	float dist = len( crossP );
//
//	return dist;
//}
//
//double PointSegmentDistance(Vector2f pt, Vector2f a, Vector2f b, Vector2f &c){
//    float t = (pt - b) DOT (a - b) / ((a - b) DOT (a - b));
//    if(t < 0) {
//        c = b;
//    }
//    else if(t > 1){
//        c = a;
//    }
//    else{
//        c = a * t + b * (1 - t);
//    }
//
//    return len(c - pt);
//}
//
//float PointPlaneDistance(Vector3f tagtPt, Vector3f point, Vector3f normal)
//{
//	Vector3f tagtvec = tagtPt - point;
//
//	float dotP = tagtvec DOT normal;
//	float dist = fabs( dotP );
//
//	return dist;
//}
//
//
//
//
////**************************************************************************************//
////                                    Projection
////**************************************************************************************//
//
//Vector3f ProjectPointToLine(Vector3f tagtPt, Vector3f linePt, Vector3f lineDir)
//{
//	lineDir = lineDir / len(lineDir);
//
//	Vector3f tagtvec = tagtPt - linePt;
//	float dotP = tagtvec DOT lineDir;
//
//	Vector3f projPt = linePt + dotP*lineDir;
//
//	return projPt;
//}
//
//Vector3f ProjectPointToPlane(Vector3f tagtPt, Plane plane)
//{
//	Vector3f vec = tagtPt - plane.point;
//	float dotP = vec DOT plane.normal;
//	Vector3f projPt = tagtPt - dotP * plane.normal;
//
//	return projPt;
//}
//
//Triangle* ProjectTriangleToPlane(Triangle *tri, Plane plane)
//{
//	Triangle *projTri = new Triangle();
//
//	for (int i=0; i<3; i++)
//	{
//		Vector3f projPt = ProjectPointToPlane(tri->v[i], plane);
//
//		projTri->v[i] = projPt;
//	}
//
//	projTri->ComputeNormal();
//	projTri->ComputeCenter();
//
//	return projTri;
//}
//
//Vector3f ProjectPointOnLine_(Vector3f linePt0, Vector3f linePt1, Vector3f tagtPt)
//{
//	Vector3f line1 = linePt1 - linePt0;
//	Vector3f line2 = tagtPt - linePt0;
//
//	float dotResult = line1 DOT line2;
//	float dist = dotResult / len(line1);
//
//	Vector3f lineDir = line1 / len(line1);
//	return linePt0 + dist*lineDir;
//}
//
//
//
//
////**************************************************************************************//
////                                    Intersection
////**************************************************************************************//
//
//double LineHitTriangle(vec rayOrg, vec rayDir, Triangle triangle, vec &hitCoord, bool isPrint)
//{
//	//double temp[3],s1[3],s2[3],s3[3],m,d1,d2,d3;
//	//double v1[3],v2[3],v3[3],y1[3],y2[3],y3[3];
//
//	double m,d1,d2,d3;
//	vec temp;
//
//	temp = triangle.v[0] - rayOrg;
//	d1 = rayDir DOT temp;
//	temp = triangle.v[1] - rayOrg;
//	d2 = rayDir DOT temp;
//	temp = triangle.v[2] - rayOrg;
//	d3 = rayDir DOT temp;
//	if (d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0)
//	{
//		if ( isPrint )
//			printf("case 0: \n");
//		return -1.0;
//	}
//
//	vec vec0 = triangle.v[2] - triangle.v[0] ;
//	vec vec1 = triangle.v[1] - triangle.v[0] ;
//	vec hitNormal = vec0 CROSS vec1;
//
//	temp = triangle.v[0] - rayOrg;
//	m = (hitNormal DOT temp) / (hitNormal DOT rayDir);
//
//	// check if hitNormal need to reverse in direction
//	if ( (hitNormal DOT rayDir) >= 0)
//	{
//		hitNormal[0] = -hitNormal[0];
//		hitNormal[1] = -hitNormal[1];
//		hitNormal[2] = -hitNormal[2];
//	}
//	hitCoord[0] = rayOrg[0] + m * rayDir[0];
//	hitCoord[1] = rayOrg[1] + m * rayDir[1];
//	hitCoord[2] = rayOrg[2] + m * rayDir[2];
//
//	if ( IsPointInsideTriangle(hitCoord, triangle, isPrint) )
//	{
//		if ( isPrint )
//			printf("Case 2: hit triangle. \n");
//		return m;
//	}
//	else
//	{
//		if ( isPrint )
//			printf("Case 1: Out triangle \n");
//		return -1.0;
//	}
//}
//
//bool IsPointInsideTriangle(vec point, Triangle triangle, bool isPrint)
//{
//	// Consider the numerical error (FLOAT_ERROR value depends on triangle scale)
//	//const float FLOAT_ERROR = 0.0000001;
//	//const float FLOAT_ERROR = 0.000001;
//	const float FLOAT_ERROR = 0.000002; // For Ring with voxelSize = 0.15
//
//	vec vec0 = triangle.v[2] - triangle.v[0] ;
//	vec vec1 = triangle.v[1] - triangle.v[0] ;
//	vec vec2 =      point  - triangle.v[0] ;
//
//	// Note: DotResult should include the length of vec2 (when point is very close to triangle.v[0]).
//	vec normal = vec0 CROSS vec1;
//	normal = normal / len(normal);
//	float dotResult = normal DOT vec2;
//
//	if ( fabs(dotResult) > FLOAT_ERROR_LARGE )
//	{
//		if( isPrint )
//		{
//			printf("Warning: The point is not on the triangle's plane. \n");
//			printf("error:  %.8f \n\n", fabs(dotResult));
//		}
//		return false;
//	}
//
//	float dot00 = vec0 DOT vec0 ;
//	float dot01 = vec0 DOT vec1 ;
//	float dot02 = vec0 DOT vec2 ;
//	float dot11 = vec1 DOT vec1 ;
//	float dot12 = vec1 DOT vec2 ;
//
//	float inverDeno = 1 / (dot00 * dot11 - dot01 * dot01) ;
//
//	float u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
//	//if (u < 0 || u > 1) // if u out of range, return directly
//	if ( u < 0-FLOAT_ERROR || u > 1+FLOAT_ERROR  )
//	{
//		if( isPrint )
//			printf("Warning: u=%.12f is out of range \n", u);
//		return false ;
//	}
//
//	float v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
//	//if (v < 0 || v > 1) // if v out of range, return directly
//	if ( v < 0-FLOAT_ERROR || v > 1+FLOAT_ERROR  )
//	{
//		if( isPrint )
//			printf("Warning: v=%.12f is out of range \n", v);
//		return false ;
//	}
//
//	if( isPrint )
//		printf( "u+v = %.12f \n", u+v);
//
//	return u + v <= 1+FLOAT_ERROR ;
//}
//
//bool IsPointInsidePlane(Vector3f point, Vector3f planePoint, Vector3f planeNormal)
//{
//	Vector3f tempVec = point - planePoint;
//	tempVec = tempVec / len(tempVec);
//
//	float dotp = tempVec DOT planeNormal;
//
//	//if( fabs(dotp) < FLOAT_ERROR_SMALL )
//	if( fabs(dotp) < FLOAT_ERROR_LARGE )
//		return true;
//	else
//		return false;
//}
//
////int TriTriIntersect(Triangle *meshTri, Triangle linkTri)
////{
////	float P[3][3];
////	float Q[3][3];
////
////	for (int j=0; j<3; j++)
////	{
////		for (int k=0; k<3; k++)
////		{
////			P[j][k]  = meshTri->v[j][k];
////			Q[j][k]  = linkTri.v[j][k];
////		}
////	}
////
////	int res = NoDivTriTriIsect(P[0], P[1], P[2], Q[0], Q[1], Q[2]);
////	//printf("res = %d \n", res );
////
////	return res;
////}
//
//bool IsBoxBoxOverlap(Vector3f boxAMinPt, Vector3f boxAMaxPt, Vector3f boxBMinPt, Vector3f boxBMaxPt)
//{
//	if (boxBMinPt.x >= boxAMaxPt.x || boxBMaxPt.x <= boxAMinPt.x ||
//		boxBMinPt.y >= boxAMaxPt.y || boxBMaxPt.y <= boxAMinPt.y)
//	{
//		return false;
//	}
//
//	else
//	{
//		return true;
//	}
//}
//
//
//
//
////**************************************************************************************//
////                                        Area
////**************************************************************************************//
//
//float GetTriangleArea(Vector3f triPt0, Vector3f triPt1, Vector3f triPt2)
//{
//	Vector3f triEdge1 = triPt1 - triPt0;
//	Vector3f triEdge2 = triPt2 - triPt0;
//
//	Vector3f crossVec = triEdge1 CROSS triEdge2;
//
//	return len(crossVec);
//}
//
//
//
//
////**************************************************************************************//
////                                  Sorting and Search
////**************************************************************************************//
//
//#define IS_LESS(v1, v2)   (v1 < v2)
//#define IS_LARGE(v1, v2)  (v1 > v2)
//#define SWAP(r,s)  do{HEAP_SORT_Type t=r; r=s; s=t; } while(0)
//
//void siftDown(vector<HEAP_SORT_Type> &Array, vector<int> &Indices, int start, int end, bool isAscend);
//
//vector<int> HeapSort(vector<HEAP_SORT_Type> &Array, bool isAscend)
//{
//	int count = Array.size();
//	int start, end;
//
//	// Initialize the index array
//	vector<int> Indices;
//	for (int i=0; i<count; i++)
//		Indices.push_back(i);
//
//	/* heapify */
//	for (start = (count-2)/2; start >=0; start--)
//	{
//		siftDown(Array, Indices, start, count, isAscend);
//	}
//
//	for (end=count-1; end > 0; end--)
//	{
//		SWAP(Array[end],   Array[0]);
//		SWAP(Indices[end], Indices[0]);
//
//		siftDown(Array, Indices, 0, end, isAscend);
//	}
//
//	return Indices;
//}
//
//void siftDown(vector<HEAP_SORT_Type> &Array, vector<int> &Indices, int start, int end, bool isAscend)
//{
//	int root = start;
//
//	while ( root*2+1 < end )
//	{
//		int child = 2*root + 1;
//
//		if ( child == Array.size()-1 )
//			return;
//
//		if ( (isAscend == true  && IS_LESS(Array[child], Array[child+1])) ||
//			 (isAscend == false && IS_LARGE(Array[child], Array[child+1])) )
//		{
//			if ( child + 1 < end )
//			{
//				child += 1;
//			}
//		}
//
//		if ( (isAscend == true  && IS_LESS(Array[root], Array[child])) ||
//			(isAscend == false && IS_LARGE(Array[root], Array[child])) )
//		{
//			SWAP(Array[child],   Array[root] );
//			SWAP(Indices[child], Indices[root]);
//
//			root = child;
//		}
//		else
//			return;
//	}
//}
//
//
//vector<int> BubbleSort(vector<float> &Array, bool isAscend)
//{
//	vector<int> Indices;
//	for (size_t i=0; i<Array.size(); i++)
//		Indices.push_back(i);
//
//	//printf("Before Sorting: ");
//	//for (int i=0; i<Array.size(); i++)
//	//	printf(" %d: %.2f ", Indices[i], Array[i]);
//	//printf("\n");
//
//	int i, j, flag = 1; // Set flag to 1 to start first pass
//	float tempValue;    // Holding variable
//	int tempIndex;      // Holding variable index
//	int num = Array.size();
//	for(i = 1; (i <= num) && flag; i++)
//	{
//		flag = 0;
//		for (j=0; j < (num -1); j++)
//		{
//			if  ( ( isAscend && Array[j+1] < Array[j]) ||
//				(!isAscend && Array[j+1] > Array[j]) )
//			{
//				// Swap the values in the array
//				tempValue = Array[j];
//				Array[j] = Array[j+1];
//				Array[j+1] = tempValue;
//
//				// Swap the index of these two values
//				tempIndex = Indices[j];
//				Indices[j] = Indices[j+1];
//				Indices[j+1] = tempIndex;
//
//				flag = 1;
//			}
//		}
//	}
//
//	//printf("After Sorting:  ");
//	//for (int i=0; i<Array.size(); i++)
//	//	printf(" %d: %.2f ", Indices[i], Array[i]);
//	//printf("\n");
//
//	return Indices;
//}
//
//vector<int> BubbleSort(vector<int> &Array, bool isAscend)
//{
//	vector<int> Indices;
//	for (size_t i=0; i<Array.size(); i++)
//		Indices.push_back(i);
//
//	//printf("Before Sorting: ");
//	//for (int i=0; i<Array.size(); i++)
//	//	printf(" %d: %.2f ", Indices[i], Array[i]);
//	//printf("\n");
//
//	int i, j, flag = 1; // Set flag to 1 to start first pass
//	float tempValue;    // Holding variable
//	int tempIndex;      // Holding variable index
//	int num = Array.size();
//	for(i = 1; (i <= num) && flag; i++)
//	{
//		flag = 0;
//		for (j=0; j < (num -1); j++)
//		{
//			if  ( ( isAscend && Array[j+1] < Array[j]) ||
//				(!isAscend && Array[j+1] > Array[j]) )
//			{
//				// Swap the values in the array
//				tempValue = Array[j];
//				Array[j] = Array[j+1];
//				Array[j+1] = tempValue;
//
//				// Swap the index of these two values
//				tempIndex = Indices[j];
//				Indices[j] = Indices[j+1];
//				Indices[j+1] = tempIndex;
//
//				flag = 1;
//			}
//		}
//	}
//
//	//printf("After Sorting:  ");
//	//for (int i=0; i<Array.size(); i++)
//	//	printf(" %d: %.2f ", Indices[i], Array[i]);
//	//printf("\n");
//
//	return Indices;
//}
//
//
////**************************************************************************************//
////                                Search within a List
////**************************************************************************************//
//
//// Note: Part could be modelID, jointID, cycleID ect.
//int GetElementIndexInList(int targtElement, vector<int> elementList)
//{
//	for (size_t i=0; i<elementList.size(); i++)
//	{
//		if ( elementList[i] == targtElement )
//			return i;
//	}
//
//	return ELEMENT_OUT_LIST;
//}
//
//int GetPointIndexInList(Vector3f tagtPoint, vector<Vector3f> pointList)
//{
//	for (size_t i = 0; i < pointList.size(); i++)
//	{
//		float dist = len(tagtPoint - pointList[i]);
//		// Note: this threshold depends on the scale of elements
//		if (dist < FLOAT_ERROR_LARGE)
//		{
//			return i;
//		}
//	}
//
//	return ELEMENT_OUT_LIST;
//}
//
//
//
//
////**************************************************************************************//
////                                Matrix Calculation
////**************************************************************************************//
//
// Note: Input matrix is an OpenGL style matrix (column major)
void MultiplyPoint(vec inPt, double inMat[16], vec &outPt)
{
	double tempVec[3] = {inPt[0], inPt[1], inPt[2]};

	double inTranMat[16];
	memcpy(inTranMat, inMat, sizeof(double)*16);
	transposeMat(inTranMat, 4);

	// Consider the translational component of the matrix
	outPt[0] = inMat[12] + dot3D(tempVec, inTranMat  );
	outPt[1] = inMat[13] + dot3D(tempVec, inTranMat+4);
	outPt[2] = inMat[14] + dot3D(tempVec, inTranMat+8);
}

//
//// Note: Input matrix is an OpenGL style matrix (column major)
//void MultiplyVector(vec inVec, double inMat[16], vec &outVec)
//{
//	double tempVec[3] = {inVec[0], inVec[1], inVec[2]};
//
//	double inTranMat[16];
//	memcpy(inTranMat, inMat, sizeof(double)*16);
//	transposeMat(inTranMat, 4);
//
//	// Ignore the translational component of the matrix
//	outVec[0] = dot3D(tempVec, inTranMat  );
//	outVec[1] = dot3D(tempVec, inTranMat+4);
//	outVec[2] = dot3D(tempVec, inTranMat+8);
//}
//
//// Note: Input matrix is an OpenGL style matrix (column major)
//void MultiplyNormal(vec inNor, double inMat[16], vec &outNor)
//{
//	double tempVec[3] = {inNor[0], inNor[1], inNor[2]};
//
//	//double inTranMat[16];
//	//memcpy(inTranMat, inMat, sizeof(double)*16);
//	//transposeMat(inTranMat, 4);
//
//	// The matrix used to transform normal is (M^-1)^T, where M is the matrix to transform vertex
//	double inveMat[16];
//	memcpy(inveMat, inMat, sizeof(double)*16);
//	if( invert4by4( inveMat ) == 0 )  printf("Inverse Matrix Error \n");
//
//	outNor[0] = dot3D(tempVec, inveMat  );
//	outNor[1] = dot3D(tempVec, inveMat+4);
//	outNor[2] = dot3D(tempVec, inveMat+8);
//
//	outNor = outNor / len(outNor);
//}
//
//// Note: Input and output matrices are OpenGL style matrix (column major)
//void MultiplyMatrix(double inLefMat[16], double inRigMat[16], double outMat[16])
//{
//	// Transpose left matrix
//	double inLefTranMat[16];
//	memcpy(inLefTranMat, inLefMat, sizeof(double)*16);
//	transposeMat(inLefTranMat, 4);
//
//	// Transpose right matrix
//	double inRigTranMat[16];
//	memcpy(inRigTranMat, inRigMat, sizeof(double)*16);
//	transposeMat(inRigTranMat, 4);
//
//	// Multiply two transposed matrices
//	multMat(inLefTranMat, inRigTranMat, outMat, 4);
//	transposeMat(outMat, 4);
//}
//
//void RotateMatrix(double angle, double x, double y, double z, double mat[16]){
//
//
////    ( xx(1-c)+c	xy(1-c)-zs  xz(1-c)+ys	 0  )
////    |					    |
////    | yx(1-c)+zs	yy(1-c)+c   yz(1-c)-xs	 0  |
////    | xz(1-c)-ys	yz(1-c)+xs  zz(1-c)+c	 0  |
////    |					    |
////    (	 0	     0		 0	 1  )
//
//    double c = std::cos(angle / 180 * M_PI);
//    double s = std::sin(angle / 180 * M_PI);
//
//
//    mat[0] =  x * x * (1 - c) + c;
//    mat[1] =  y * x * (1 - c) + z * s;
//    mat[2] =  x * z * (1 - c) - y * s;
//    mat[3] =  0;
//
//    mat[4] =  x * y * (1 - c) - z * s;
//    mat[5] =  y * y * (1 - c) + c;
//    mat[6] =  y * z * (1 - c) + x * s;
//    mat[7] =  0;
//
//    mat[8] =  x * z * (1 - c) + y * s;
//    mat[9] =  y * z * (1 - c) - x * s;
//    mat[10] = z * z * (1 - c) + c;
//    mat[11] = 0;
//
//    mat[12] = 0;
//    mat[13] = 0;
//    mat[14] = 0;
//    mat[15] = 1;
//}
//
//void ScaleMatrix(double x, double y, double z, double mat[16]){
//    mat[0] =  x;
//    mat[1] =  0;
//    mat[2] =  0;
//    mat[3] =  0;
//
//    mat[4] =  0;
//    mat[5] =  y;
//    mat[6] =  0;
//    mat[7] =  0;
//
//    mat[8] =  0;
//    mat[9] =  0;
//    mat[10] = z;
//    mat[11] = 0;
//
//    mat[12] = 0;
//    mat[13] = 0;
//    mat[14] = 0;
//    mat[15] = 1;
//}
//
//void TranslateMatrix(double x, double y, double z, double mat[16]){
//    mat[0] =  1;
//    mat[1] =  0;
//    mat[2] =  0;
//    mat[3] =  0;
//
//    mat[4] =  0;
//    mat[5] =  1;
//    mat[6] =  0;
//    mat[7] =  0;
//
//    mat[8] =  0;
//    mat[9] =  0;
//    mat[10] = 1;
//    mat[11] = 0;
//
//    mat[12] = x;
//    mat[13] = y;
//    mat[14] = z;
//    mat[15] = 1;
//}
//
//void LoadIdentityMatrix(double mat[16]){
//    mat[0] =  1;
//    mat[1] =  0;
//    mat[2] =  0;
//    mat[3] =  0;
//
//    mat[4] =  0;
//    mat[5] =  1;
//    mat[6] =  0;
//    mat[7] =  0;
//
//    mat[8] =  0;
//    mat[9] =  0;
//    mat[10] = 1;
//    mat[11] = 0;
//
//    mat[12] = 0;
//    mat[13] = 0;
//    mat[14] = 0;
//    mat[15] = 1;
//}
//
//bool EqualMatrix(double matA[16], double matB[16])
//{
//	bool isEqual = true;
//	for (int i=0; i<16; i++)
//	{
//		if ( matA[i] != matB[i] )
//		{
//			isEqual = false;
//			break;
//		}
//	}
//
//	return isEqual;
//}
//
//void PrintMatrix(double matrix[])
//{
//	for (int i=0; i<16; i++)
//	{
//		printf("%8.5f ", matrix[i]);
//		if( (i+1)%4 == 0 )
//			printf("\n");
//	}
//	printf("\n");
//}
//
//void PrintVector(vector<Vector3f> points)
//{
//	printf("PointNum: %d \n", (int)points.size());
//
//	for (size_t i = 0; i < points.size(); i++)
//	{
//		printf("[%6.3f  %6.3f  %6.3f] \n", points[i].x, points[i].y, points[i].z);
//	}
//	printf("\n");
//}
//
//
//
//
////**************************************************************************************//
////                                  Transform Matrix
////**************************************************************************************//
//
//void GetRotationMatrix(float rotAngle, Vector3f rotAxis, double rotMat[16])
//{
//	float a = rotAngle *M_PI / 180.0;
//	float x = rotAxis.x;
//	float y = rotAxis.y;
//	float z = rotAxis.z;
//
//	// Rotation matrix is an OpenGL-style matrix
//	rotMat[0] = cos(a) + x*x*(1 - cos(a));
//	rotMat[1] = y*x*(1 - cos(a)) + z*sin(a);
//	rotMat[2] = z*x*(1 - cos(a)) - y*sin(a);
//	rotMat[3] = 0;
//
//	rotMat[4] = x*y*(1 - cos(a)) - z*sin(a);
//	rotMat[5] = cos(a) + y*y*(1 - cos(a));
//	rotMat[6] = z*y*(1 - cos(a)) + x*sin(a);
//	rotMat[7] = 0;
//
//	rotMat[8] = x*z*(1 - cos(a)) + y*sin(a);
//	rotMat[9] = y*z*(1 - cos(a)) - x*sin(a);
//	rotMat[10] = cos(a) + z*z*(1 - cos(a));
//	rotMat[11] = 0;
//
//	rotMat[12] = 0;
//	rotMat[13] = 0;
//	rotMat[14] = 0;
//	rotMat[15] = 1;
//}
//
//void GetRotationMatrix(float rotAngle, Vector3f rotAxis,  Vector3f rotCen, double rotMat[16])
//{
//	double lefTransMat[16];
//	identityd(lefTransMat);
//
//	lefTransMat[12] = rotCen.x;
//	lefTransMat[13] = rotCen.y;
//	lefTransMat[14] = rotCen.z;
//
//	double rigTransMat[16];
//	identityd(rigTransMat);
//
//	rigTransMat[12] = -rotCen.x;
//	rigTransMat[13] = -rotCen.y;
//	rigTransMat[14] = -rotCen.z;
//
//	GetRotationMatrix(rotAngle, rotAxis, rotMat);
//
//	MultiplyMatrix(lefTransMat, rotMat, rotMat);
//	MultiplyMatrix(rotMat, rigTransMat, rotMat);
//}
//
//void GetPlaneTransformMatrix(Vector3f planeNormal, Vector3f planeVec, double transMat[16])
//{
//	planeNormal = planeNormal / len(planeNormal);
//	planeVec = planeVec / len(planeVec);
//	Vector3f perpVec = planeVec CROSS planeNormal;
//
//	identityd(transMat);
//
//	transMat[0] = planeVec.x;  // X-axis
//	transMat[4] = planeVec.y;
//	transMat[8] = planeVec.z;
//
//	transMat[1] = perpVec.x;  // Y-axis
//	transMat[5] = perpVec.y;
//	transMat[9] = perpVec.z;
//
//	transMat[2] = planeNormal.x;  // Z-axis
//	transMat[6] = planeNormal.y;
//	transMat[10] = planeNormal.z;
//}
//
//Vector3f RotateVector(Vector3f rotCenter, Vector3f rotAxis, float rotAngle, Vector3f tagtPt)
//{
//	// Get the rotation matrix using OpenGL modelView matrix
//
//	Eigen::Matrix4d trans1 = Eigen::Matrix4d::Identity();
//	trans1(0, 3) = rotCenter.x; trans1(1, 3) = rotCenter.y; trans1(2, 3) = rotCenter.z;
//
//	Eigen::Matrix4d rotate = Eigen::Matrix4d::Identity();
//	double x, y, z, c, s;
//	x = rotAxis.x; y = rotAxis.y; z = rotAxis.z;
//	c = std::cos(rotAngle / 180 * M_PI);
//	s = std::sin(rotAngle / 180 * M_PI);
//	rotate << c + x * x *(1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s, 0,
//			  y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s, 0,
//			  z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c), 0,
//			  0, 0, 0, 1;
//
//	Eigen::Matrix4d trans2 = Eigen::Matrix4d::Identity();
//	trans2(0, 3) = -rotCenter.x; trans2(1, 3) = -rotCenter.y; trans2(2, 3) = -rotCenter.z;
//
//	Eigen::Vector4d pos;
//	pos << tagtPt.x, tagtPt.y, tagtPt.z, 1;
//
//	Eigen::Vector4d result = trans1 * rotate * trans2 * pos;
//	//std::cout << result[0] << ", " << result[1] << ", " <<  result[2] << std::endl;
//	return Vector3f(result[0], result[1], result[2]);
//}
//
////**************************************************************************************//
////                                  String and Time
////**************************************************************************************//
//
//void GetFolderPath(const char filePath[], char folderPath[])
//{
//	string sFilePath = string(filePath);
//
//	// Subtract file folder path
//	size_t foundFolder = sFilePath.find_last_of("\\");
//	string sFolderPath = sFilePath.substr(0, foundFolder);
//	sprintf(folderPath, "%s", sFolderPath.c_str());
//
//	//printf( puzFolderPath );
//	//printf("\n");
//}
//
//void GetFileNameNoExtension(const char filePath[], char fileName[])
//{
//	string sFilePath = string(filePath);
//
//	// Subtract file name (without folder path)
//	int foundName = sFilePath.find_last_of("\\");
//	int foundType = sFilePath.find_last_of(".");
//
//	if (foundName == -1) foundName = 0;                        // In case no "\\" in file Path
//	if (foundType == -1) foundType = sFilePath.size() - 1;     // In case no "."  in file Path
//
//	string sFileName = sFilePath.substr(foundName, foundType);
//	sprintf(fileName, "%s", sFileName.c_str());
//}
//
//string GetFileType(const char filePath[])
//{
//	string sFilePath = string(filePath);
//
//	// Subtract file extension type
//	size_t stringLen = sFilePath.size();
//	size_t foundType = sFilePath.find_last_of(".");
//	string sFileType = sFilePath.substr(foundType + 1, stringLen - (foundType + 1));
//
//	return sFileType;
//}
//
//string GetLocalTime()
//{
//	time_t rawtime;
//	struct tm * timeinfo;
//	char buffer[80];
//
//	time(&rawtime);
//	timeinfo = localtime(&rawtime);
//
//	//strftime(buffer, 80, "%d-%m-%Y %I:%M:%S", timeinfo);
//	strftime(buffer, 80, "%Y-%m-%d %I:%M", timeinfo);
//	std::string str(buffer);
//
//	//printf( str.c_str() );
//	//printf("\n");
//
//	return str;
//}
//
//
//
//
////**************************************************************************************//
////                                  Read/Write Files
////**************************************************************************************//
//
//void WriteOBJModel(char *objFileName, vector<Vector3f> verList, vector<Vector3i> triList)
//{
//
//	FILE *fp;
//	if ((fp = fopen(objFileName, "w+")) == NULL)
//	{
//		printf("Error: file not exists! \n");
//		return;
//	}
//	else
//	{
//		////////////////////////////////////////////////////////
//		// Write the vertex info of piece OBJ Part
//
//		fprintf(fp, "# Wavefront OBJ generated by Peng SONG \n\n");
//
//		fprintf(fp, "# %d vertices \n", (int)verList.size());
//
//		for (size_t i = 0; i < verList.size(); i++)
//		{
//			fprintf(fp, "v %f %f %f \n", verList[i].x, verList[i].y, verList[i].z);
//		}
//		fprintf(fp, "\n");
//
//		////////////////////////////////////////////////////////
//		// Write the quad info of piece OBJ Part
//
//		fprintf(fp, "# %d faces \n", (int)triList.size());
//
//		for (size_t i = 0; i < triList.size(); i++)
//		{
//			fprintf(fp, "f ");
//			for (int j = 0; j < 3; j++)
//			{
//				fprintf(fp, " %d", triList[i][j] + 1);
//			}
//			fprintf(fp, "\n");
//
//			//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
//			//fprintf(fp, "f %d %d %d \n", tri->vIndices[0] + 1, tri->vIndices[1] + 1, tri->vIndices[2] + 1);
//		}
//		fprintf(fp, "\n");
//
//		fclose(fp);
//	}
//}
//
//
//
//#if USE_OPENGL_DRAW
////**************************************************************************************//
////                               Draw Geometric Primitive
////**************************************************************************************//
//
//void DrawPoints(vector<Vector3f> points, float size, Vector3f color)
//{
//	glDisable(GL_LIGHTING);
//	glPointSize(size);
//
//	glColor3f(color.x, color.y, color.z);
//	glBegin(GL_POINTS);
//	for (size_t i = 0; i < points.size(); i++)
//	{
//		glVertex3f(points[i].x, points[i].y, points[i].z);
//	}
//	glEnd();
//
//	glPointSize(size);
//	glEnable(GL_LIGHTING);
//}
//
//void DrawPolygon(vector<Vector3f> poly, float width, Vector3f color)
//{
//	glDisable(GL_LIGHTING);
//	glLineWidth(width);
//
//	glColor3f(color.x, color.y, color.z);
//	glBegin(GL_POLYGON);
//	for (size_t i = 0; i < poly.size(); i++)
//	{
//		glVertex3f(poly[i].x, poly[i].y, poly[i].z);
//	}
//	glEnd();
//
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
//}
//
//void DrawLine(Vector3f start_point, Vector3f end_point, float width, Vector3f color)
//{
//	glDisable(GL_LIGHTING);
//	glLineWidth(width);
//
//	glColor3f(color.x, color.y, color.z);
//	glBegin(GL_LINE_STRIP);
//	glVertex3f(start_point.x, start_point.y, start_point.z);
//	glVertex3f(end_point.x, end_point.y, end_point.z);
//	glEnd();
//
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
//}
//
//void DrawSolidCuboid(Vector3f minPt, Vector3f maxPt, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission)
//{
//	glEnable(GL_LIGHTING);
//
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
//	glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION,  emission);
//	GLfloat shininess[] = { 76.8f };
//	glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, shininess);
//
//	glBegin(GL_QUADS);
//	glNormal3f( 0, 0, -1);
//	glVertex3f(minPt.x, minPt.y, minPt.z); // 0
//	glNormal3f( 0, 0, -1);
//	glVertex3f(minPt.x, maxPt.y, minPt.z); // 3
//	glNormal3f( 0, 0, -1);
//	glVertex3f(maxPt.x, maxPt.y, minPt.z); // 2
//	glNormal3f( 0, 0, -1);
//	glVertex3f(maxPt.x, minPt.y, minPt.z); // 1
//
//	glNormal3f( 0, 0, 1);
//	glVertex3f(minPt.x, minPt.y, maxPt.z); // 4
//	glNormal3f( 0, 0, 1);
//	glVertex3f(maxPt.x, minPt.y, maxPt.z); // 5
//	glNormal3f( 0, 0, 1);
//	glVertex3f(maxPt.x, maxPt.y, maxPt.z); // 6
//	glNormal3f( 0, 0, 1);
//	glVertex3f(minPt.x, maxPt.y, maxPt.z); // 7
//
//	glNormal3f( 0, -1, 0);
//	glVertex3f(minPt.x, minPt.y, minPt.z); // 0
//	glNormal3f( 0, -1, 0);
//	glVertex3f(maxPt.x, minPt.y, minPt.z); // 1
//	glNormal3f( 0, -1, 0);
//	glVertex3f(maxPt.x, minPt.y, maxPt.z); // 5
//	glNormal3f( 0, -1, 0);
//	glVertex3f(minPt.x, minPt.y, maxPt.z); // 4
//
//	glNormal3f( 0, 1, 0);
//	glVertex3f(minPt.x, maxPt.y, minPt.z); // 3
//	glNormal3f( 0, 1, 0);
//	glVertex3f(minPt.x, maxPt.y, maxPt.z); // 7
//	glNormal3f( 0, 1, 0);
//	glVertex3f(maxPt.x, maxPt.y, maxPt.z); // 6
//	glNormal3f( 0, 1, 0);
//	glVertex3f(maxPt.x, maxPt.y, minPt.z); // 2
//
//	glNormal3f( -1, 0, 0);
//	glVertex3f(minPt.x, minPt.y, minPt.z); // 0
//	glNormal3f( -1, 0, 0);
//	glVertex3f(minPt.x, minPt.y, maxPt.z); // 4
//	glNormal3f( -1, 0, 0);
//	glVertex3f(minPt.x, maxPt.y, maxPt.z); // 7
//	glNormal3f( -1, 0, 0);
//	glVertex3f(minPt.x, maxPt.y, minPt.z); // 3
//
//	glNormal3f( 1, 0, 0);
//	glVertex3f(maxPt.x, minPt.y, minPt.z); // 1
//	glNormal3f( 1, 0, 0);
//	glVertex3f(maxPt.x, maxPt.y, minPt.z); // 2
//	glNormal3f( 1, 0, 0);
//	glVertex3f(maxPt.x, maxPt.y, maxPt.z); // 6
//	glNormal3f( 1, 0, 0);
//	glVertex3f(maxPt.x, minPt.y, maxPt.z); // 5
//
//	glEnd();
//}
//
//void DrawWireCuboid(Vector3f minPt, Vector3f maxPt, float lineWidth, Vector3f color)
//{
//	glDisable(GL_LIGHTING);
//	glColor4f(color.x, color.y, color.z, 0.9f);
//	glLineWidth(lineWidth);
//
//	glBegin(GL_LINE_LOOP);
//	glVertex3f(minPt.x, minPt.y, minPt.z); // 0
//	glVertex3f(maxPt.x, minPt.y, minPt.z); // 1
//	glVertex3f(maxPt.x, maxPt.y, minPt.z); // 2
//	glVertex3f(minPt.x, maxPt.y, minPt.z); // 3
//	glEnd();
//
//	glBegin(GL_LINE_LOOP);
//	glVertex3f(minPt.x, minPt.y, maxPt.z); // 4
//	glVertex3f(minPt.x, maxPt.y, maxPt.z); // 7
//	glVertex3f(maxPt.x, maxPt.y, maxPt.z); // 6
//	glVertex3f(maxPt.x, minPt.y, maxPt.z); // 5
//	glEnd();
//
//	glBegin(GL_LINE_LOOP);
//	glVertex3f(minPt.x, minPt.y, minPt.z); // 0
//	glVertex3f(minPt.x, minPt.y, maxPt.z); // 4
//	glVertex3f(maxPt.x, minPt.y, maxPt.z); // 5
//	glVertex3f(maxPt.x, minPt.y, minPt.z); // 1
//	glEnd();
//
//	glBegin(GL_LINE_LOOP);
//	glVertex3f(minPt.x, maxPt.y, minPt.z); // 3
//	glVertex3f(maxPt.x, maxPt.y, minPt.z); // 2
//	glVertex3f(maxPt.x, maxPt.y, maxPt.z); // 6
//	glVertex3f(minPt.x, maxPt.y, maxPt.z); // 7
//	glEnd();
//
//	glEnable(GL_LIGHTING);
//	glLineWidth(1.0);
//}
//
//void DrawSphere(Vector3f position, float radius, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission)
//{
//	//glEnable(GL_LIGHTING);
//
//	float mtlAmbient[4]  = { ambient.x, ambient.y, ambient.z, 1.0 };
//	float mtlDiffuse[4]  = { diffuse.x, diffuse.y, diffuse.z, 1.0 };
//	float mtlSpecular[4] = { specular.x, specular.y, specular.z, 1.0 };
//	float mtlEmission[4] = { emission.x, emission.y, emission.z, 1.0 };
//
//	glPushAttrib(GL_LIGHTING_BIT);
//
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mtlAmbient);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mtlDiffuse);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mtlSpecular);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mtlEmission);
//	GLfloat shininess[] = { 76.8f };
//	glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, shininess);
//
//	gluQuadricDrawStyle(quadObj, GLU_FILL);
//	gluQuadricNormals(quadObj, GLU_SMOOTH);
//
//	glPushMatrix();
//	glTranslatef(position.x, position.y, position.z);
//	// TODO: memory leak issue of glutSolidSphere
//	//glutSolidSphere(radius, 8, 4);
//	glutSolidSphere(radius, 36, 18);
//	glPopMatrix();
//
//	//glDisable(GL_LIGHTING);
//
//	glPopAttrib();
//}
//
//void DrawWireSphere(Vector3f center, float radius, float lineWidth)
//{
//	glLineWidth(lineWidth);
//	glDisable(GL_LIGHTING);
//
//	glPushMatrix();
//	glTranslatef(center.x, center.y, center.z);
//	//glutWireSphere(radius, 8, 4);
//	glutWireSphere(radius, 16, 8);
//	glPopMatrix();
//
//	glEnable(GL_LIGHTING);
//	glLineWidth(1.0);
//}
//
//void DrawCylinder(Vector3f p1, Vector3f p2, float radius, Vector3f ambient, Vector3f diffuse, Vector3f specular, Vector3f emission)
//{
//	glEnable(GL_LIGHTING);
//
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
//	glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION,  emission);
//	GLfloat shininess[] = { 76.8f };
//	glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, shininess);
//
//	float vx = p2.x - p1.x;
//	float vy = p2.y - p1.y;
//	float vz = p2.z - p1.z;
//
//	//handle the degenerate case with an approximation
//	if (vz == 0) vz = .00000001;
//
//	float v = sqrt(vx * vx + vy * vy + vz * vz);
//	float ax = 57.2957795 * acos(vz / v);
//
//	if ( vz < 0.0 ) ax = -ax;
//	float rx = -vy * vz;
//	float ry =  vx * vz;
//
//	gluQuadricDrawStyle(quadObj, GLU_FILL);
//	gluQuadricNormals(quadObj, GLU_SMOOTH);
//
//	glPushMatrix();
//	glTranslatef(p1.x, p1.y, p1.z);
//	glRotatef(ax, rx, ry, 0.0);
//	gluCylinder(quadObj, radius, radius, v, 8, 1);
//	//gluCylinder(quadObj, radius, radius, v, 24, 1);
//	//gluQuadricOrientation(q,GLU_INSIDE);
//	//draw the first cap
//	//gluDisk( q, 0.0, radius, 32, 1);
//	//glTranslatef( 0,0,v );
//	//draw the second cap
//	//gluQuadricOrientation(q,GLU_OUTSIDE);
//	//gluDisk( q, 0.0, radius, 32, 1);
//	glPopMatrix();
//}
//
//
//float Draw2DTextAt(char *str, float offsetX, float offsetY, float size, int alignment, int spaceShift, int pointSize)
//{
//	float pixelShift;
//	int   i;
//
//	///////////////////////////////////////////
//	// 1. Need the width of text?
//
//	pixelShift = 0.0;
//
//	// Additional space shift?
//	pixelShift += glutStrokeWidth(GLUT_STROKE_ROMAN,'-') * spaceShift;
//
//	///////////////////////////////////////////
//	// 2. Reset Matrices
//#define SCREEN_RATIO 1.6
//
//	// reset the projection
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity();
//	gluOrtho2D(0.0,SCREEN_RATIO,0.0,1.0);
//
//	// reset the modelview
//	glMatrixMode(GL_MODELVIEW);
//	glPushMatrix();
//	glLoadIdentity();
//	glTranslatef(offsetX*SCREEN_RATIO+pixelShift*size,1.0-offsetY,0.0f);
//	glScalef(size,size,size);
//
//	// Note: draw the point before the text with specified pointSize
//	if( pointSize > 0 )
//	{
//		glPointSize(pointSize);
//		glBegin(GL_POINTS);
//		glVertex3f(-100.0, 50.0, 0.0);
//		glEnd();
//		glPointSize(2.0);
//	}
//
//	///////////////////////////////////////////
//	// 3. Render the text
//
//	glDepthFunc(GL_ALWAYS);
//	for (i=0; i<((int)strlen(str)); i++) {
//		if (str[i] != ' ')
//			glutStrokeCharacter(GLUT_STROKE_ROMAN,str[i]);
//
//		else
//			glTranslatef(200000*size,0.0,0.0);
//	}
//	glDepthFunc(GL_LESS);
//
//	///////////////////////////////////////////
//	// 4. Reset the matrices
//
//	glMatrixMode(GL_PROJECTION);
//	glPopMatrix();
//	glMatrixMode(GL_MODELVIEW);
//	glPopMatrix();
//
//	return pixelShift ;
//}
//
//
//void Draw3DText(Vector3f texPosition, float textScale, float lineWidth, int textNumber)
//{
//	glLineWidth(lineWidth);
//	glDisable(GL_LIGHTING);
//
//	glMatrixMode(GL_MODELVIEW);
//
//	glPushMatrix();
//	glTranslatef(texPosition.x, texPosition.y, texPosition.z);
//	glScalef(textScale, textScale, textScale);
//
//	vector<int> allDigits = GetNumberDigits(textNumber);
//
//	Vector3f transVec;
//	for (size_t i = 0; i < allDigits.size(); i++)
//	{
//		//transVec = Vector3f(0.1f*i, 0.1f*i, 0.1f*i);
//		transVec = Vector3f(2.6f, 0, 0);
//		glTranslatef(transVec.x, transVec.y, transVec.z);
//
//		Draw3DDigit(allDigits[i]);
//	}
//
//	glPopMatrix();
//
//	glEnable(GL_LIGHTING);
//	glLineWidth(1.0f);
//}
//
//vector<int> GetNumberDigits(int number)
//{
//	vector<int> allDigits;
//
//	const int maxDigitNum = 6;
//
//	int currNumber = number;
//	for (int i = maxDigitNum; i > 0; i--)
//	{
//		int digit = (int) (currNumber / pow(10, i));
//
//		if ( digit > 0 || number >= pow(10, i))
//		{
//			allDigits.push_back(digit);
//
//			currNumber -= digit * pow(10, i);
//		}
//	}
//
//	int unitDigit = number % 10;
//	allDigits.push_back(unitDigit);
//
//	return allDigits;
//}
//
//void Draw3DDigit(int digit)
//{
//	const float r = 1.5;
//
//	Vector3f a = Vector3f(-1,  0, 0);
//	Vector3f b = Vector3f( 1,  0, 0);
//	Vector3f c = Vector3f(-1,  r, 0);
//	Vector3f d = Vector3f( 1,  r, 0);
//	Vector3f e = Vector3f(-1, -r, 0);
//	Vector3f f = Vector3f( 1, -r, 0);
//
//	glBegin(GL_LINES);
//
//	if (digit == 2 || digit == 3 || digit == 4 || digit == 5 || digit == 6 || digit == 8 || digit == 9)
//	{
//		glVertex3f(a.x, a.y, a.z);
//		glVertex3f(b.x, b.y, b.z);
//	}
//
//	if (digit == 0 || digit == 2 || digit == 3 || digit == 5 || digit == 6 || digit == 7 || digit == 8 || digit == 9)
//	{
//		glVertex3f(c.x, c.y, c.z);
//		glVertex3f(d.x, d.y, d.z);
//	}
//
//	if (digit == 0 || digit == 1 || digit == 4 || digit == 5 || digit == 6 || digit == 8 || digit == 9)
//	{
//		glVertex3f(c.x, c.y, c.z);
//		glVertex3f(a.x, a.y, a.z);
//	}
//
//	if (digit == 0 || digit == 2 || digit == 3 || digit == 4 || digit == 7 || digit == 8 || digit == 9)
//	{
//		glVertex3f(d.x, d.y, d.z);
//		glVertex3f(b.x, b.y, b.z);
//	}
//
//	if (digit == 0 || digit == 2 || digit == 3 || digit == 5 || digit == 6 || digit == 8 || digit == 9)
//	{
//		glVertex3f(e.x, e.y, e.z);
//		glVertex3f(f.x, f.y, f.z);
//	}
//
//	if (digit == 0 || digit == 1 || digit == 2 || digit == 6 || digit == 8)
//	{
//		glVertex3f(a.x, a.y, a.z);
//		glVertex3f(e.x, e.y, e.z);
//	}
//
//	if (digit == 0 || digit == 3 || digit == 4 || digit == 5 || digit == 6 || digit == 7 || digit == 8 || digit == 9)
//	{
//		glVertex3f(b.x, b.y, b.z);
//		glVertex3f(f.x, f.y, f.z);
//	}
//
//	glEnd();
//}
//
//
//void DrawPlane(Vector3f point, Vector3f normal, float radius, Vector3f color)
//{
//	Vector3f linePt = point + 0.3f * normal;
//
//	Vector3f localX = normal CROSS Vector3f(0, 0, 1);
//	//Vector3f localX = normal CROSS Vector3f(0, 1, 0);
//	localX = localX / len(localX);
//	Vector3f localY = normal CROSS localX;
//
//	int polyPtNum = 4;
//
//	glDisable(GL_LIGHTING);
//	//glPointSize(12.0);
//	//glLineWidth(3.0);
//
//	//glColor3f(0.9, 0.6, 0.3);
//	//glBegin(GL_POINTS);
//	//glVertex3f(point.x, point.y, point.z);
//	//glEnd();
//
//	//glColor3f(0.9, 0.6, 0.3);
//	//glBegin(GL_LINES);
//	//glVertex3f(point.x, point.y, point.z);
//	//glVertex3f(linePt.x, linePt.y, linePt.z);
//	//glEnd();
//
//	glEnable(GL_LIGHTING);
//
//	float ambient[4] = { 0.3, 0.3, 0.3, 1.0 };
//	float specular[4] = { color.x, color.y, color.z, 1.0 };
//	float diffuse[4] = { color.x, color.y, color.z, 1.0 };
//	float emission[4] = { 0.2, 0.2, 0.2, 1.0 };
//
//	glPushAttrib(GL_LIGHTING_BIT);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
//	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
//
//	glColor3f(0.6, 0.6, 0.6);
//	glBegin(GL_POLYGON);
//	for (int i = 0; i < polyPtNum; i++)
//	{
//		float theta = i * 2 * M_PI / (float)polyPtNum;
//
//		Vector3f polyPt = point + radius*(localX*cos(theta) + localY*sin(theta));
//
//		glNormal3f(normal.x, normal.y, normal.z);
//		glVertex3f(polyPt.x, polyPt.y, polyPt.z);
//	}
//	glEnd();
//
//	glPopAttrib();
//
//	//glPointSize(1.0);
//	//glLineWidth(1.0);
//}
//
//#endif
//
//Vector3f ColorMapping(float min, float max, float value)
//{
//	Vector3f blue   = Vector3f(0.2, 0.2, 0.9);
//	Vector3f cyan   = Vector3f(0.2, 0.9, 0.9);
//	Vector3f green  = Vector3f(0.2, 0.9, 0.2);
//	Vector3f yellow = Vector3f(0.9, 0.9, 0.2);
//	Vector3f red    = Vector3f(0.9, 0.2, 0.2);
//
//	float dot1 = (max + 3 * min) / 4;
//	float dot2 = (max + min) / 2;
//	float dot3 = (3*max + min) / 4;
//
//	Vector3f color;
//	if (value <= min)        color = blue;
//	else if (value < dot1)   color = GetColor(min, dot1, value, blue, cyan);
//	else if (value < dot2)   color = GetColor(dot1, dot2, value, cyan, green);
//	else if (value < dot3)   color = GetColor(dot2, dot3, value, green, yellow);
//	else if (value < max)    color = GetColor(dot3, max, value, yellow, red);
//	else                     color = red;
//
//	return color;
//}
//
//Vector3f GetColor(float min, float max, float value, Vector3f minColor, Vector3f maxColor)
//{
//	float r = (value - min) / (max - min);
//
//	Vector3f color = (1-r)*minColor + r*maxColor;
//
//	return color;
//}