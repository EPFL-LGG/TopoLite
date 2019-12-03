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




#ifndef __MATH3D_
#define __MATH3D_

#include <stdio.h>
//#define _DEBUG


#ifndef _EPSILON
#define _EPSILON 1e-7
#endif

#ifndef _TRUE
#define _TRUE 1
#endif

#ifndef _FALSE
#define _FALSE 0
#endif


#ifdef __cplusplus
extern "C" {
#endif


void printVec(const double *v, int c);


// check if trangle v1,v2,v3 is anticlockwise w.r.t. +z
extern int anticlockwise2D(const double *v1, const double *v2, const double *v3);

extern double clamp(const double val1);

extern double dist2D(const double *pt1, const double *pt2);
extern double dist3D(const double *pt1, const double *pt2);

extern double length2D(const double *pt1);
extern double length3D(const double *pt1);

extern double sqrLength2D(const double *pt1);
extern double sqrLength3D(const double *pt1);

extern void add2D(const double *vect1, const double *vect2, double *vect3);
extern void add3D(const double *vect1, const double *vect2, double *vect3);

extern void minus2D(const double *vect1, const double *vect2, double *vect3);
extern void minus3D(const double *vect1, const double *vect2, double *vect3);

extern void add_mul_3D(const double *vect1, const double t, const double *vect2, double *v1_add_t_mul_v2);
extern void add_mul_2D(const double *vect1, const double t, const double *vect2, double *v1_add_t_mul_v2);

extern double dot2D(const double *vect1, const double *vect2);
extern double dot3D(const double *vect1, const double *vect2);

// v cross w = r
extern void cross3D(const double *v, const double *w, double *r);

extern void copy2D(double *vect1, const double *vect2);
extern void copy3D(double *vect1, const double *vect2);

extern int equalVec2D(const double *vect1, const double *vect2);
extern int equalVec3D(const double *vect1, const double *vect2);
extern int equalVec2D_Ep(const double *vect1, const double *vect2, double epsilon);
extern int equalVec3D_Ep(const double *vect1, const double *vect2, double epsilon);

extern void mult2D(double *vect, const double scale);
extern void mult3D(double *vect, const double scale);

extern void div2D(double *vect, const double scale);
extern void div3D(double *vect, const double scale);

extern void interp2D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 ) ;
extern void interp3D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 ) ;
extern void interp4D ( double * result , double * vect0 , double * vect1 , double weight0 , double weight1 ) ;

extern int unify2D(double *vect);
extern int unify3D(double *vect);

extern void findNormal3D(const double *pt1, const double *pt2, const double *pt3, double *normal);
extern void average3D(const double *plane1, const double *plane2, const double *plane3, double *normal);

extern double hitTriangle(const double *ver1,   const double *ver2, const double *ver3,
                          const double *rayOrg, const double *rayDir,
                          double *hitCoord, double *hitNormal);
extern double hitSphere(const double *center, const double radius,
                        const double *rayOrg, const double *rayDir,
                        double *hitCoord, double *hitNormal);

extern int anticlockwise2D(const double *v1, const double *v2, const double *v3);

extern int areParallelVec2D(const double *vect1, const double *vect2);
extern int areParallelVec3D(const double *vect1, const double *vect2);

// angle is in radian
extern void rotate2D(double *outVec, const double *inVec, const double angle);
extern void rotate3D_3(double *inOutVec, const double *axis, const double angle);


extern void printMat( FILE *stream, const char *format, const double *D, const int row, const int column );
extern void transposeMat( double *D, const int n );
extern void multMat( const double *A, const double *B, double *result, const int n );

// cast 4x4 matrix to be identity and 4x4 matrix inversion
extern void identityd    ( double   m[16] ) ;
extern int  invert4by4  ( double src[16] ) ;
extern void identityf   ( float    m[16] ) ;
extern int  invert4by4f ( float  src[16] ) ;

// invert the square matrix where D is a n by n row-wise matrix
// return 1 on error else 0
extern int invertMat( double *D, const int n );


#ifdef __cplusplus
}
#endif


#endif
