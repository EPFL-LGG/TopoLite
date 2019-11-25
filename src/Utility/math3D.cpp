///////////////////////////////////////////////////////////////////////
//
// Mathematics Library for 3D
//
// math3D.h
//
// by Philip Fu (cwfu@acm.org)
// Copyright 2002, Trustees of Indiana University
//
// Mon Jan 22 7:54:04 EST 2001
//
// All rights reserved
//
///////////////////////////////////////////////////////////////////////


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "math3D.h"   /* my math library header file */



#ifdef _DEBUG
#include <assert.h>
#endif




void
printVec(const double *v, int c)
{
    int i = 0;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(v);
    #endif

    printf("%7.5f",v[i++]);
    if (i < c)
	do {
	    printf(" %7.5f",v[i++]);
	} while (i < c);
}


/*** clamp the value from 0 to 1 ***/
double clamp(const double val1)
{
    if (val1 < 0.0)
	return 0.0;
    else if (val1 > 1.0)
	return 1.0;
    else
	return val1;
}


/*** find 2D distance */
double dist2D(const double *pt1, const double *pt2)
{
    double dx,dy;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
      assert(pt2);
    #endif

    dx = pt1[0]-pt2[0];
    dy = pt1[1]-pt2[1];

    return sqrt(dx*dx+dy*dy);
}

/*** find 3D distance */
double dist3D(const double *pt1, const double *pt2)
{
    double dx,dy,dz;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
      assert(pt2);
    #endif

    dx = pt1[0]-pt2[0];
    dy = pt1[1]-pt2[1];
    dz = pt1[2]-pt2[2];

    return sqrt(dx*dx+dy*dy+dz*dz);
}


/*** sqr. length of a 2D vector */
double sqrLength2D(const double *pt1)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
    #endif

    return (pt1[0]*pt1[0]+pt1[1]*pt1[1]);
}


/*** sqr. length of a 3D vector */
double sqrLength3D(const double *pt1)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
    #endif

    return (pt1[0]*pt1[0]+pt1[1]*pt1[1]+pt1[2]*pt1[2]);
}


/*** length of a 2D vector */
double length2D(const double *pt1)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
    #endif

    return (sqrt(pt1[0]*pt1[0]+pt1[1]*pt1[1]));
}


/*** length of a 3D vector */
double length3D(const double *pt1)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
    #endif

    return (sqrt(pt1[0]*pt1[0]+pt1[1]*pt1[1]+pt1[2]*pt1[2]));
}


/*** two vectors' difference is another vector ***/
void minus2D(const double *vect1, const double *vect2, double *vect3)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
      assert(vect3);
    #endif

    vect3[0] = vect1[0] - vect2[0];
    vect3[1] = vect1[1] - vect2[1];
}


/*** two vectors' difference is another vector ***/
void minus3D(const double *vect1, const double *vect2, double *vect3)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
      assert(vect3);
    #endif

    vect3[0] = vect1[0] - vect2[0];
    vect3[1] = vect1[1] - vect2[1];
    vect3[2] = vect1[2] - vect2[2];
}


/*** two vectors' sum is another vector ***/
void add2D(const double *vect1, const double *vect2, double *vect3)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
      assert(vect3);
    #endif

    vect3[0] = vect1[0] + vect2[0];
    vect3[1] = vect1[1] + vect2[1];
}


/*** two vectors' sum is another vector ***/
void add3D(const double *vect1, const double *vect2, double *vect3)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
      assert(vect3);
    #endif

    vect3[0] = vect1[0] + vect2[0];
    vect3[1] = vect1[1] + vect2[1];
    vect3[2] = vect1[2] + vect2[2];
}


/*** add and multiply ***/
void
add_mul_2D(const double *vect1, const double t, const double *vect2, double *v1_add_t_mul_v2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    v1_add_t_mul_v2[0] = vect1[0] + t * vect2[0];
    v1_add_t_mul_v2[1] = vect1[1] + t * vect2[1];
}


/*** add and multiply ***/
void
add_mul_3D(const double *vect1, const double t, const double *vect2, double *v1_add_t_mul_v2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif
    
    v1_add_t_mul_v2[0] = vect1[0] + t * vect2[0];
    v1_add_t_mul_v2[1] = vect1[1] + t * vect2[1];
    v1_add_t_mul_v2[2] = vect1[2] + t * vect2[2];
}


/*** calculate the dot product of two vectors ***/
double dot2D(const double *vect1, const double *vect2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    return (vect1[0]*vect2[0] + vect1[1]*vect2[1]);
}


/*** calculate the dot product of two vectors ***/
double dot3D(const double *vect1, const double *vect2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    return (vect1[0]*vect2[0] + vect1[1]*vect2[1] + vect1[2]*vect2[2]);
}


/*** calculate the plane cross product : v cross w = r ***/
void cross3D(const double *v, const double *w, double *r)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(v);
      assert(w);
      assert(r);
    #endif

    r[0] = v[1]*w[2] - v[2]*w[1];
    r[1] = v[2]*w[0] - v[0]*w[2];
    r[2] = v[0]*w[1] - v[1]*w[0];
}


/*** copy vect1 from vect2 ***/
void copy2D(double *vect1, const double *vect2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    vect1[0] = vect2[0];
    vect1[1] = vect2[1];
}


/*** copy vect1 from vect2 ***/
void copy3D(double *vect1, const double *vect2)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    vect1[0] = vect2[0];
    vect1[1] = vect2[1];
    vect1[2] = vect2[2];
}


/*** multiply a vector by a factor***/
void mult2D(double *vect, const double scale)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    vect[0] = vect[0]*scale;
    vect[1] = vect[1]*scale;
}


/*** multiply a vector by a factor***/
void mult3D(double *vect, const double scale)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    vect[0] = vect[0]*scale;
    vect[1] = vect[1]*scale;
    vect[2] = vect[2]*scale;
}


/*** divede a vector by a factor***/
void div2D(double *vect, const double scale)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    vect[0] = vect[0]/scale;
    vect[1] = vect[1]/scale;
}


/*** divede a vector by a factor***/
void div3D(double *vect, const double scale)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    vect[0] = vect[0]/scale;
    vect[1] = vect[1]/scale;
    vect[2] = vect[2]/scale;
}


/*** interpolation in 2D ***/
void interp2D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 )
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(result);
      assert(vect0);
      assert(vect1);
    #endif

    result[0] = vect0[0] * weight0 + vect1[0] * weight1 ;
    result[1] = vect0[1] * weight0 + vect1[1] * weight1 ;
}


/*** interpolation in 3D ***/
void interp3D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 )
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(result);
      assert(vect0);
      assert(vect1);
    #endif

    result[0] = vect0[0] * weight0 + vect1[0] * weight1 ;
    result[1] = vect0[1] * weight0 + vect1[1] * weight1 ;
    result[2] = vect0[2] * weight0 + vect1[2] * weight1 ;
}


/*** interpolation in 4D ***/
void interp4D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 )
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(result);
      assert(vect0);
      assert(vect1);
    #endif

    result[0] = vect0[0] * weight0 + vect1[0] * weight1 ;
    result[1] = vect0[1] * weight0 + vect1[1] * weight1 ;
    result[2] = vect0[2] * weight0 + vect1[2] * weight1 ;
    result[3] = vect0[3] * weight0 + vect1[3] * weight1 ;
}


/*** copy vect1 from vect2 ***/
int equalVec2D_Ep(const double *vect1, const double *vect2, double epsilon)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    if (fabs (vect1[0]-vect2[0]) > epsilon)
	return _FALSE;
    if (fabs (vect1[1]-vect2[1]) > epsilon)
	return _FALSE;
    return _TRUE;
}


/*** copy vect1 from vect2 ***/
int equalVec3D_Ep(const double *vect1, const double *vect2, double epsilon)
{
    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    if (fabs (vect1[0]-vect2[0]) > epsilon)
	return _FALSE;
    if (fabs (vect1[1]-vect2[1]) > epsilon)
	return _FALSE;
    if (fabs (vect1[2]-vect2[2]) > epsilon)
	return _FALSE;
    return _TRUE;
}


/*** copy vect1 from vect2 ***/
int equalVec2D(const double *vect1, const double *vect2)
{
    return equalVec2D_Ep(vect1,vect2,_EPSILON);
}


/*** copy vect1 from vect2 ***/
int equalVec3D(const double *vect1, const double *vect2)
{
    return equalVec3D_Ep(vect1,vect2,_EPSILON);
}


/*** unify a vector ***/
int unify2D(double *vect)
{
    double length;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    length = sqrt(vect[0]*vect[0]+vect[1]*vect[1]);
    if (length == 0.0)
	return 0;

    vect[0] = vect[0] / length;
    vect[1] = vect[1] / length;

    return 1;
}


/*** unify a vector ***/
int unify3D(double *vect)
{
    double length;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect);
    #endif

    length = sqrt(vect[0]*vect[0]+vect[1]*vect[1]+vect[2]*vect[2]);

    if (length == 0.0)
	return 0;

    vect[0] = vect[0] / length;
    vect[1] = vect[1] / length;
    vect[2] = vect[2] / length;

    return 1;
}


/*** calculate the plane normal ***/

// pt1, pt2 and pt3 in anticlockwise order

void findNormal3D(const double *pt1, const double *pt2, const double *pt3, double *normal)
{
    double vec12[3],vec13[3];

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(pt1);
      assert(pt2);
      assert(pt3);
      assert(normal);
    #endif

    minus3D(pt2,pt1,vec12);
    minus3D(pt3,pt1,vec13);
    cross3D(vec12,vec13,normal); 
}


/*** find the vertex normal from the surrounding plane normals */
/*** with unification ***/

void
average3D(const double *plane1, const double *plane2, const double *plane3, double *normal)
{
    double modulus;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(plane1);
      assert(plane2);
      assert(plane3);
      assert(normal);
    #endif

    normal[0] = plane1[0]+plane2[0]+plane3[0];
    normal[1] = plane1[1]+plane2[1]+plane3[1];
    normal[2] = plane1[2]+plane2[2]+plane3[2];

    /* unified the normal */
    modulus = sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
    normal[0] /= modulus;
    normal[1] /= modulus;
    normal[2] /= modulus;
}


/* check if a ray (ray origin and ray direction)
            hit a triangle (3 vertices)
         then, save hit coordinate and the normal
         if hit, return the light ray ratio m,
            otherwise, -1.0,
*/

double
hitTriangle(const double *ver1,     const double *ver2,   const double *ver3,
            const double *rayOrg,   const double *rayDir,
            double *hitCoord, double *hitNormal)
{
    double temp[3],s1[3],s2[3],s3[3],m,d1,d2,d3;
    double v1[3],v2[3],v3[3],y1[3],y2[3],y3[3];

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(ver1);
      assert(ver2);
      assert(ver3);
      assert(rayOrg);
      assert(rayDir);
      assert(hitCoord);
      assert(hitNormal);
    #endif

    minus3D(ver1,rayOrg,temp);
    d1 = dot3D(rayDir,temp);
    minus3D(ver2,rayOrg,temp);
    d2 = dot3D(rayDir,temp);
    minus3D(ver3,rayOrg,temp);
    d3 = dot3D(rayDir,temp);
    if (d1 <= 0.0 && d2 <= 0.0 && d3 <= 0.0)
	return -1.0;

    findNormal3D(ver1,ver2,ver3,hitNormal);

    minus3D(ver1,rayOrg,temp);
    m = dot3D(hitNormal,temp) / dot3D(hitNormal,rayDir);

	/* check if hitNormal need to reverse in direction */
	if (dot3D(hitNormal,rayDir) >= 0) {
	    hitNormal[0] = -hitNormal[0];
	    hitNormal[1] = -hitNormal[1];
	    hitNormal[2] = -hitNormal[2];
	}
	hitCoord[0] = rayOrg[0] + m * rayDir[0];
	hitCoord[1] = rayOrg[1] + m * rayDir[1];
	hitCoord[2] = rayOrg[2] + m * rayDir[2];

	/* check if hitCoord is within triangle boundary */
	minus3D(hitCoord,ver1,s1);
	minus3D(hitCoord,ver2,s2);
	minus3D(hitCoord,ver3,s3);
	minus3D(ver2,ver1,v1);
	minus3D(ver3,ver2,v2);
	minus3D(ver1,ver3,v3);
	cross3D(s1,v1,y1);
	cross3D(s2,v2,y2);
	cross3D(s3,v3,y3);

        if (  (dot3D(y1,y2) > 0.0)
           && (dot3D(y2,y3) > 0.0)
           && (dot3D(y3,y1) > 0.0) )
	    return m;
	else
	    return -1.0;
}


/* check if a ray (ray origin and ray direction)
            hit a sphere (center and radius)
         then, save hit coordinate and the normal
         if hit, return light ray ratio m,
             otherwise, -1.0,
*/

double
hitSphere(const double *center, const double radius,
          const double *rayOrg, const double *rayDir,
          double *hitCoord, double *hitNormal)
{
    double A,B,C,discriminant,t,m1,m2,temp[3],origin[3];


    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(center);
      assert(rayOrg);
      assert(rayDir);
      assert(hitCoord);
      assert(hitNormal);
    #endif


    origin[0] = origin[1] = origin[2] = 0;
    A = pow(dist3D(rayDir,origin),2);
    minus3D(rayOrg,center,temp);
    B = 2*dot3D(rayDir,temp);
    C = pow(dist3D(temp,origin),2) - radius*radius;

    discriminant = B*B-4*A*C;

    if (discriminant < 0)
	return -1.0;

    t  = sqrt(discriminant);
    m1 = (-B + t) / (2*A);
    m2 = (-B - t) / (2*A);

    if (m1 > 0 && m2 > 0) {

	/* take the smallest ratio to be hit point ratio */
	if (m2 < m1)
	    m1 = m2;

	hitCoord[0] = rayOrg[0] + m1*rayDir[0];
	hitCoord[1] = rayOrg[1] + m1*rayDir[1];
	hitCoord[2] = rayOrg[2] + m1*rayDir[2];
	minus3D(hitCoord,center,hitNormal);
	unify3D(hitNormal);

	return m1;
    } else
	return -1.0;
}


// check if trangle v1,v2,v3 is anticlockwise w.r.t. +z

int
anticlockwise2D(const double *v1, const double *v2, const double *v3)
{
    double vec12[3],vec13[3],cross[3];

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(v1);
      assert(v2);
      assert(v3);
    #endif

    vec12[0] = v2[0] - v1[0];
    vec12[1] = v2[1] - v1[1];
    if (vec12[0] == 0.0 && vec12[1] == 0.0)
	return -1;
    vec12[2] = 0.0;

    vec13[0] = v3[0] - v1[0];
    vec13[1] = v3[1] - v1[1];
    if (vec13[0] == 0.0 && vec13[1] == 0.0)
	return -1;
    vec13[2] = 0.0;

    cross3D(vec12,vec13,cross);
    if (cross[2] > 0.0)
	return _TRUE;
    else if (cross[2] < 0.0)
	return _FALSE;
    else
	return -1;
}


int
areParallelVec2D(const double *vect1, const double *vect2)
{
    double a,b;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    a = vect1[0] * vect2[1];
    b = vect1[1] * vect2[0];

    if (fabs(a-b) < _EPSILON)
	return _TRUE;
    else
	return _FALSE;
}


int
areParallelVec3D(const double *vect1, const double *vect2)
{
    double a,b,c;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(vect1);
      assert(vect2);
    #endif

    a = vect1[0] * vect2[1] * vect2[2];
    b = vect1[1] * vect2[2] * vect2[0];
    c = vect1[2] * vect2[0] * vect2[1];

    if (fabs(a-b) < _EPSILON && fabs(b-c) < _EPSILON)
	return _TRUE;
    else
	return _FALSE;
}


// angle is in radian

void
rotate2D(double *outVec, const double *inVec, const double angle)
{
    double c,s;

    // no assertion in math3D.c to keep the program efficiency
    #ifdef _DEBUG
      assert(outVec);
      assert(inVec);
    #endif

    c = cos(angle);
    s = sin(angle);

    outVec[0] = c * inVec[0] - s * inVec[1];
    outVec[1] = s * inVec[0] + c * inVec[1];
}


// angle is in radian

void
rotate3D_3(double *inOutVec, const double *axis, const double angle)
{
    double c,s,outVec[3],mat[9],myaxis[3];

    // form the rotation matrix
    c = cos(angle);
    s = sin(angle);

    copy3D(myaxis,axis);
    unify3D(myaxis);

	#define n1  (myaxis[0])
	#define n2  (myaxis[1])
	#define n3  (myaxis[2])

    mat[0] = c + n1*n1*(1-c);
    mat[1] = n1*n2*(1-c) - s*n3;
    mat[2] = n3*n1*(1-c) + s*n2;

    mat[3] = n1*n2*(1-c) + s*n3;
    mat[4] = c + n2*n2*(1-c);
    mat[5] = n3*n2*(1-c) - s*n1;

    mat[6] = n3*n1*(1-c) - s*n2;
    mat[7] = n3*n2*(1-c) + s*n1;
    mat[8] = c + n3*n3*(1-c);

    // perform rotation
    outVec[0] = dot3D(mat,  inOutVec);
    outVec[1] = dot3D(mat+3,inOutVec);
    outVec[2] = dot3D(mat+6,inOutVec);

    copy3D(inOutVec,outVec);
}


void
printMat( FILE *stream, const char *format, const double *D, const int row, int const column )
{
    int i,j,p;

    p = 0;
    for (j=0; j<row; j++) {
	fprintf(stream,format,D[p++]);
	for (i=1; i<column; i++) {
	    fprintf(stream," ");
	    fprintf(stream,format,D[p++]);
	}
	fprintf(stream,"\n");
    }
}


void
transposeMat( double *D, const int n )
{
    int i,j,p,q;

    p = 0;

    for (j=0; j<n; j++)
	for (i=0; i<n; i++) {

	    if (i > j) {

		double tmp;

		q    = i*n+j;
		tmp  = D[p];
		D[p] = D[q];
		D[q] = tmp;

		//printf("Swap %d and %d\n",p,q);
	    }

	    p++;
	}
}


void
multMat( const double *A, const double *B, double *result, const int n )
{
    int i,j,k,p;

    p = 0;

    // ith row in A
    for (i=0; i<n; i++)

	// jth column in B
	for (j=0; j<n; j++) {

	    // result[i,j]
	    result[p] = 0.0;
	    for (k=0; k<n; k++)
		result[p] += A[i*n+k] * B[k*n+j];

	    p++;
	}
}


void
identityd(double m[16])
{
    m[0] = 1; m[4] = 0; m[8]  = 0; m[12] = 0;
    m[1] = 0; m[5] = 1; m[9]  = 0; m[13] = 0;
    m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
    m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
}


void
identityf(float m[16])
{
    m[0] = 1.0f; m[4] = 0.0f; m[8]  = 0.0f; m[12] = 0.0f;
    m[1] = 0.0f; m[5] = 1.0f; m[9]  = 0.0f; m[13] = 0.0f;
    m[2] = 0.0f; m[6] = 0.0f; m[10] = 1.0f; m[14] = 0.0f;
    m[3] = 0.0f; m[7] = 0.0f; m[11] = 0.0f; m[15] = 1.0f;
}


// More Secure version for inverse of 4 x 4 matrix
// From:
// - projection.c
//   Nate Robins, 1997

int
invert4by4(double src[16])
{
    double t;
    int i, j, k, swap;
    double tmp[4][4];
    double inverse[16];

    identityd(inverse);

    for (i = 0; i < 4; i++) {
	for (j = 0; j < 4; j++) {
	    tmp[i][j] = src[i*4+j];
	}
    }

    for (i = 0; i < 4; i++) {
        /* look for largest Part in column. */
        swap = i;
        for (j = i + 1; j < 4; j++) {
            if (fabs(tmp[j][i]) > fabs(tmp[i][i])) {
                swap = j;
            }
        }

        if (swap != i) {
            /* swap rows. */
            for (k = 0; k < 4; k++) {
                t = tmp[i][k];
                tmp[i][k] = tmp[swap][k];
                tmp[swap][k] = t;

                t = inverse[i*4+k];
                inverse[i*4+k] = inverse[swap*4+k];
                inverse[swap*4+k] = t;
            }
        }

        if (tmp[i][i] == 0) {
            /* no non-zero pivot.  the matrix is singular, which
	       shouldn't happen.  This means the user gave us a bad
	       matrix. */
            return 0;
        }

        t = tmp[i][i];
        for (k = 0; k < 4; k++) {
            tmp[i][k] /= t;
            inverse[i*4+k] /= t;
        }
        for (j = 0; j < 4; j++) {
            if (j != i) {
                t = tmp[j][i];
                for (k = 0; k < 4; k++) {
                    tmp[j][k] -= tmp[i][k]*t;
                    inverse[j*4+k] -= inverse[i*4+k]*t;
                }
            }
        }
    }

    // done!
    memcpy(src,inverse,sizeof(double)*16);

    return 1;
}


// More Secure version for inverse of 4 x 4 matrix
// From:
// - projection.c
//   Nate Robins, 1997

int
invert4by4f(float src[16])
{
    float t;
    int   i, j, k, swap;
    float tmp[4][4];
    float inverse[16];

    identityf(inverse);

    for (i = 0; i < 4; i++) {
	for (j = 0; j < 4; j++) {
	    tmp[i][j] = src[i*4+j];
	}
    }

    for (i = 0; i < 4; i++) {
        /* look for largest Part in column. */
        swap = i;
        for (j = i + 1; j < 4; j++) {
            if (fabs(tmp[j][i]) > fabs(tmp[i][i])) {
                swap = j;
            }
        }

        if (swap != i) {
            /* swap rows. */
            for (k = 0; k < 4; k++) {
                t = tmp[i][k];
                tmp[i][k] = tmp[swap][k];
                tmp[swap][k] = t;

                t = inverse[i*4+k];
                inverse[i*4+k] = inverse[swap*4+k];
                inverse[swap*4+k] = t;
            }
        }

        if (tmp[i][i] == 0) {
            /* no non-zero pivot.  the matrix is singular, which
	       shouldn't happen.  This means the user gave us a bad
	       matrix. */
            return 0;
        }

        t = tmp[i][i];
        for (k = 0; k < 4; k++) {
            tmp[i][k] /= t;
            inverse[i*4+k] /= t;
        }
        for (j = 0; j < 4; j++) {
            if (j != i) {
                t = tmp[j][i];
                for (k = 0; k < 4; k++) {
                    tmp[j][k] -= tmp[i][k]*t;
                    inverse[j*4+k] -= inverse[i*4+k]*t;
                }
            }
        }
    }

    // done!
    memcpy(src,inverse,sizeof(float)*16);

    return 1;
}


/* invert the square matrix */
/* D is a n by n row-wise matrix */

// From http://uk.geocities.com/sanliuk/matrix_inversion.html

int invertMat( double *D, const int n )
{
    int l,m,p;
    int pos,p_old;
    int p_old2;

    for (l=0;l<n;l++) {

	p_old2    = l*n+l;
	D[p_old2] = 1 / D[p_old2];

	for (m=0; m<n; m++)
	    if (m!=l) {
		p_old    = m*n+l;
		D[p_old] = D[p_old]*D[p_old2];
	    }

	for (m=0; m<n; m++) {
	    for (p=0; p<n; p++) {
		if (m!=l) {
		    if (p!=l) {
			pos    = m*n+p;
			p_old  = m*n+l;
			p_old2 = l*n+p;
			D[pos] = D[pos] - (D[p_old]*D[p_old2]);
		    }
		}
	    }
	} 

	p_old = l*n+l;

	for (p=0; p<n; p++)
	    if (p!=l) {
		pos=l*n+p; 
		D[pos]=-D[p_old]*D[pos]; 
	    }
    }

    return 0;
}

