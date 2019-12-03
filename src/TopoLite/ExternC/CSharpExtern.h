//
// Created by ziqwang on 2019-10-28.
//

#ifndef  C_SHARP_EXTERN_INCLUDE
#define C_SHARP_EXTERN_INCLUDE

#include "CSharpCommon.h"
#include "Structure/StrucCreator.h"
#include "IO/XMLIO.h"

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
XMLData* readXML(const char *xmlstr);

CSharp_LIBRARY_C_FUNCTION
int deleteStructure(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void refresh(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void preview(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
int partNumber(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
bool initMesh(int partID, CMesh *mesh, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
bool assignMesh(int partID, CMesh *mesh, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
bool isBoundary(int partID, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
float ComputeGroundHeight(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setParaDouble(const char *name, double value, XMLData* data);

#endif
