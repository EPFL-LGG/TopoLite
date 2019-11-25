///////////////////////////////////////////////////////////////
//
// Controls.h
//
//   Defined Variables to Control the Program
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 15/July/2018
//
///////////////////////////////////////////////////////////////


#ifndef _CONTROLS
#define _CONTROLS


//#define DEBUG_CODE_PENG           0

#define SURFACE_CylinderA60       1
#define SURFACE_CylinderA80       2
#define SURFACE_HyperbolicA90     3
#define SURFACE_SeaShell          4
#define SURFACE_SphereA60         5
#define SURFACE_Torus             6
#define SURFACE_Spindle           7


#define DBG_DOTP_EPSILON          0.0001


//**************************************************************************************//
//                                Control Parameters 
//**************************************************************************************//

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


//#define CROSS_TRI                0
#define CROSS_SQUARE                       1
#define CROSS_RHOMBUS                      2
#define CROSS_SQUARE_RHOMBUS               3
#define CROSS_HEXAGON                      4
#define CROSS_HEXAGON_RHOMBUS              5
#define CROSS_DODECAGON_HEXAGON_QUAD       6
#define CROSS_OCTAGON_SQUARE               7

#define CROSS_OCTAGON_SQUARE_COLINEAR      8
#define CROSS_DODECAGON                    9
#define CROSS_PENTAGON_CROSS              10
#define CROSS_PENTAGON_SNOW               11
#define CROSS_PENTAGON_MIRROR             12



#define POLY_SQUARE_THETA_45               1
#define POLY_SQUARE_THETA_15               2
#define POLY_SQUARE_THETA_75               3
#define POLY_RHOMBUS_THETA_0               4
#define POLY_RHOMBUS_THETA_90              5
#define POLY_RHOMBUS_THETA_120             6
#define POLY_RHOMBUS_THETA_240             7
#define POLY_HEXAGON_TYPE_0                8
#define POLY_HEXAGON_TYPE_1                9
#define POLY_OCTAGON_REGULAR              10
#define POLY_OCTAGON_COLINEAR             11
#define POLY_DODECAGON                    12
#define POLY_PENTAGON_CROSS_TYPE_0        13
#define POLY_PENTAGON_CROSS_TYPE_1        14
#define POLY_PENTAGON_CROSS_TYPE_2        15
#define POLY_PENTAGON_CROSS_TYPE_3        16
#define POLY_PENTAGON_SNOW_TYPE_0         17
#define POLY_PENTAGON_SNOW_TYPE_1         18
#define POLY_PENTAGON_SNOW_TYPE_2         19
#define POLY_PENTAGON_SNOW_TYPE_3         20
#define POLY_PENTAGON_SNOW_TYPE_4         21
#define POLY_PENTAGON_SNOW_TYPE_5         22
#define POLY_PENTAGON_MIRROR_TYPE_0       23
#define POLY_PENTAGON_MIRROR_TYPE_1       24
#define POLY_RHOMBUS_TYPE_0               25
#define POLY_RHOMBUS_TYPE_1               26

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
