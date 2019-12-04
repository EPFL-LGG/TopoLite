//
// Created by ziqwang on 2019-10-28.
//

#ifndef  C_SHARP_EXTERN_INCLUDE
#define C_SHARP_EXTERN_INCLUDE

#include "CSharpCommon.h"
#include "Structure/StrucCreator.h"
#include "Mesh/MeshConverter.h"
#include "IO/XMLIO.h"

//CSharp_LIBRARY C Exports
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
PolyMeshRhino *initPolyMeshRhino(int partID, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
bool isNull(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
bool deletePolyMeshRhino(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNVertices(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNFaces(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
void copyVertexI(PolyMeshRhino *mesh, int vID, float *point);

CSharp_LIBRARY_C_FUNCTION
void copyFaceI(PolyMeshRhino *mesh, int fID, int *face);

CSharp_LIBRARY_C_FUNCTION
int getNVertexGroup(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNFaceGroup(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNVertexGroupI(PolyMeshRhino *mesh, int vgID);

CSharp_LIBRARY_C_FUNCTION
int getNFaceGroupI(PolyMeshRhino *mesh, int vfID);

CSharp_LIBRARY_C_FUNCTION
void copyVertexGroupI(PolyMeshRhino *mesh, int vgID, int* vg);

CSharp_LIBRARY_C_FUNCTION
void copyFaceGroupI(PolyMeshRhino *mesh, int fgID, int* fg);

CSharp_LIBRARY_C_FUNCTION
bool isBoundary(int partID, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
float ComputeGroundHeight(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setParaDouble(const char *name, double value, XMLData* data);

#endif
