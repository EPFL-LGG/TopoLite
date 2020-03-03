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
#ifndef UNITTEST_DATAPATH
    #define UNITTEST_DATAPATH
#endif
///////////////////////////////////////////////////////////////////////
//
//  Debug Flag
//

//#define _DEBUG


///////////////////////////////////////////////////////////////////////
//  General Definition

// List
#define ELEMENT_OUT_LIST            -1

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

//**************************************************************************************//
//                                Control Parameters
//**************************************************************************************//

//#define DEBUG_CODE_PENG           0

#define SURFACE_CylinderA60       1
#define SURFACE_CylinderA80       2
#define SURFACE_HyperbolicA90     3
#define SURFACE_SeaShell          4
#define SURFACE_SphereA60         5
#define SURFACE_Torus             6
#define SURFACE_Spindle           7


#define DBG_DOTP_EPSILON          0.0001

//#define CO_PLANAR_THRES          0.000001
//#define CO_PLANAR_THRES          0.00001
#define CO_PLANAR_THRES          0.001

#define MODEL_HEXAGON            1
#define MODEL_HEXAGON_5x5        2
#define MODEL_HEXAGON_10         3
#define MODEL_SQUARE             4
#define MODEL_SQUARE_5X5         5
#define MODEL_SQUARE_10          6

// Generating 2D tiling pattern
//#define CROSS_RING_NUM           40
#define CROSS_L                  0.4

#define MOBILITY_SCORE_MAX       3.0
#define MOBILITY_SCORE_DIFF      0.0001

#define NONE_CROSS              -1
#define NONE_PART               -1
#define NONE_GROUP              -1
#define NONE_ELEMENT            -1
#define NONE_FACE               -1

#define MOBILITY_SPHERE_R        3.0f


#define CROSS_THETA             35.264389     // theta = asin(sqrt(3.0) / 3.0) * (180 / M_PI)

#define TILT_SIGN_NONE           0
#define TILT_SIGN_POSITIVE       1
#define TILT_SIGN_NEGATIVE      -1


//**************************************************************************************//
//                                Control Parameters
//**************************************************************************************//

// Initial World Pose
#define INIT_WORLD_POSITION            Vector3f(0.0,  -0.0, -3.8)
#define INIT_WORLD_ROT_AXIS            Vector3f(1.0,  0.0,  0.0)
#define INIT_WORLD_ROT_ANGLE           30.0
#define INIT_WORLD_SCALE               1.0

// Initial World Axes Pose
#define INIT_WORLD_AXES_POSITION       Vector3f(0.0,  0.0, -4.0)
#define INIT_WORLD_AXES_ROT_AXIS       Vector3f(1.0,  0.0,  0.0)
#define INIT_WORLD_AXES_ROT_ANGLE      15.0


//**************************************************************************************//
//                                    Definitions
//**************************************************************************************//

// DRAW Mode
#define DRAW_POINT             0
#define DRAW_LINE              1
#define DRAW_FLAT              2
#define DRAW_SMOOTH            3

#endif
