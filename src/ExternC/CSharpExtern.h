//
// Created by ziqwang on 2019-10-28.
//

#ifndef  C_SHARP_EXTERN_INCLUDE
#define C_SHARP_EXTERN_INCLUDE

#include "CSharpCommon.h"

typedef struct CPoint{
    float x, y, z;
}CPoint;

typedef struct CFace
{
    int a, b, c;
}CFace;

typedef struct CMesh
{
    CPoint* points;
    CFace* faces;
    int n_vertices;
    int n_faces;
}CMesh;

// CSharp_LIBRARY C Exports
CSharp_LIBRARY_C_FUNCTION
int readXML(const char *xmlstr);

CSharp_LIBRARY_C_FUNCTION
int deleteStructure();

CSharp_LIBRARY_C_FUNCTION
void refresh();

CSharp_LIBRARY_C_FUNCTION
void preview();

CSharp_LIBRARY_C_FUNCTION
int partNumber();

CSharp_LIBRARY_C_FUNCTION
bool initMesh(int partID, CMesh *mesh);

CSharp_LIBRARY_C_FUNCTION
bool assignMesh(int partID, CMesh *mesh);

CSharp_LIBRARY_C_FUNCTION
bool isBoundary(int partID);

CSharp_LIBRARY_C_FUNCTION
float ComputeGroundHeight();

CSharp_LIBRARY_C_FUNCTION
void setParaDouble(const char *name, double value);

#endif
