//
// Created by ziqwang on 2019-10-28.
//

#ifndef  C_SHARP_EXTERN_INCLUDE
#define C_SHARP_EXTERN_INCLUDE

#include "CSharpCommon.h"
#include "Structure/StrucCreator.h"
#include "Mesh/MeshConverter.h"
#include "Interlocking/ContactGraph.h"
#include "IO/XMLIO.h"
struct CMesh{
    float* points;
    int* faces;
    int n_vertices;
    int n_faces;
};

struct CPolyLines
{
    float* points;
    int* sta_ends;
    int* atBoundary;
    int n_polyline;
    int n_points;
};

struct ContactGraphData{
    vector<pPolyMesh> meshes;
    vector<bool> atBoundary;
    shared_ptr<ContactGraph> graph;
};

//IO
CSharp_LIBRARY_C_FUNCTION
XMLData* readXML(const char *xmlstr);

CSharp_LIBRARY_C_FUNCTION
XMLData* initStructure();

CSharp_LIBRARY_C_FUNCTION
ContactGraphData* initContactGraph();

CSharp_LIBRARY_C_FUNCTION
PolyMeshRhino *initPartMeshPtr(int partID, XMLData *data);

CSharp_LIBRARY_C_FUNCTION
PolyLineRhino *initCrossMeshPtr(XMLData *data);

CSharp_LIBRARY_C_FUNCTION
PolyLineRhino *initTextureMeshPtr(XMLData *data);

CSharp_LIBRARY_C_FUNCTION
PolyLineRhino *initPatternMeshPtr(XMLData *data);

CSharp_LIBRARY_C_FUNCTION
PolyLineRhino *initBaseMesh2DPtr(XMLData *data);

CSharp_LIBRARY_C_FUNCTION
PolyMeshRhino *initContactMesh(ContactGraphData *data);

CSharp_LIBRARY_C_FUNCTION
int testInterlocking(ContactGraphData *data);

CSharp_LIBRARY_C_FUNCTION
int deleteStructure(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
int deleteContactGraph(ContactGraphData* data);

CSharp_LIBRARY_C_FUNCTION
int deletePolyMeshRhino(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int deletePolyLineRhino(PolyLineRhino *lines);


//Create Geometry
CSharp_LIBRARY_C_FUNCTION
void refresh(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void preview(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void addMeshesToContactGraph(ContactGraphData *data, CMesh *cmesh, bool brdy);

//Get Info
CSharp_LIBRARY_C_FUNCTION
int partNumber(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
int isNull(void *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNVertices(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNFaces(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNVertexGroup(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int getNFaceGroup(PolyMeshRhino *mesh);

CSharp_LIBRARY_C_FUNCTION
int isBoundary(int partID, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
float ComputeGroundHeight(XMLData* data);

CSharp_LIBRARY_C_FUNCTION
int getNPolyLines(PolyLineRhino *polylines);

CSharp_LIBRARY_C_FUNCTION
int getNPolyLineI(PolyLineRhino *polylines, int lID);

CSharp_LIBRARY_C_FUNCTION
int getNVertexGroupI(PolyMeshRhino *mesh, int vgID);

CSharp_LIBRARY_C_FUNCTION
int getNFaceGroupI(PolyMeshRhino *mesh, int vfID);


//Copy Data
CSharp_LIBRARY_C_FUNCTION
void copyVertexI(PolyMeshRhino *mesh, int vID, float *point);

CSharp_LIBRARY_C_FUNCTION
void copyFaceI(PolyMeshRhino *mesh, int fID, int *face);

CSharp_LIBRARY_C_FUNCTION
void copyPolyLineI(PolyLineRhino *polylines, int lID, float *points);

CSharp_LIBRARY_C_FUNCTION
void copyVertexGroupI(PolyMeshRhino *mesh, int vgID, int* vg);

CSharp_LIBRARY_C_FUNCTION
void copyFaceGroupI(PolyMeshRhino *mesh, int fgID, int* fg);


//Set Para
CSharp_LIBRARY_C_FUNCTION
void setCrossMesh(CPolyLines *polylines, XMLData* data, bool haveBoundary);

CSharp_LIBRARY_C_FUNCTION
void setPattern(CPolyLines *polylines, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setReferenceSurface(CMesh *cmesh, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setParaDouble(const char *name, double value, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setParaInt(const char *name, int value, XMLData* data);

CSharp_LIBRARY_C_FUNCTION
void setPatternAngle(double angle, XMLData *data);

CSharp_LIBRARY_C_FUNCTION
void setPatternXY(double x, double y, XMLData *data);

CSharp_LIBRARY_C_FUNCTION
void setPatternScale(double s, XMLData *data);


#endif
