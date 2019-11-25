///////////////////////////////////////////////////////////////
//
// Utility/HelpDefine.h
//
//   Utility Definitions
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 25/Aug/2015
//
//
///////////////////////////////////////////////////////////////


#ifndef _HELP_DEFINE_H
#define _HELP_DEFINE_H

#include <math.h>


///////////////////////////////////////////////////////////////////////
//
//  Debug Flag
//

//#define _DEBUG


///////////////////////////////////////////////////////////////////////
//  General Definition

// File and buffer
#define FILE_NAME_LENGTH            1024
#define BUFFER_LENGTH               256

// Max and min integer/float
#define  MIN_INT                   -10000000
#define  MAX_INT                    10000000
#define  MIN_FLOAT                 -10000000.0
#define  MAX_FLOAT                  10000000.0

#define FLOAT_ERROR_SMALL           1e-7
#define FLOAT_ERROR_LARGE           1e-5
#define CLIPPER_INTERGER_SCALE      1e8f


#define errexit(s)		{ fprintf(stderr,s);     exit(EXIT_FAILURE); }
#define errexit2(s1,s2)		{ fprintf(stderr,s1,s2); exit(EXIT_FAILURE); }

#define _MAX_STR_SIZE		512
#define _MAX_LINE_SIZE		512
#define _MAX_PATH_SIZE		512

#define _ERROR			-1
#define _ERROR1			-2
#define _ERROR2			-3
#define _OKAY			 1

#define _TRUE			1
#define _FALSE			0

///////////////////////////////////////////////////////////////////////
// Math Related Definition

#define _TOGGLE(bool)		( ((bool) == _TRUE) ? _FALSE : _TRUE )

#define _ToRadian(X)		((X)/180.0*M_PI)
#define _ToDegree(X)		((X)*180.0/M_PI)

#ifndef _EPSILON
#define _EPSILON		1e-7
#endif

#ifndef _MAX
#define _MAX(a,b)		( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef _MIN
#define _MIN(a,b)		( ((a) < (b)) ? (a) : (b) )
#endif

#ifndef _IN_BETWEEN
#define _IN_BETWEEN(v,a,b)	( ((v) >= (a)) && ((v) <= (b)) )
#endif

#ifndef M_PI
#define M_PI			3.1415926535897932384626433832795
#endif

#endif
