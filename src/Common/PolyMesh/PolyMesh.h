///////////////////////////////////////////////////////////////
//
// PolyMesh.h
//
//   Polygonal Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 09/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _POLY_MESH_H
#define _POLY_MESH_H

#ifdef WIN32
#include <GL/glut.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif


#include "../HelpStruct.h"


using namespace std;

class Triangle;
class _Polygon;

class PolyMesh
{
public:
	vector<Vector3d> vertexList;         // List of vertex positions
	vector<Vector2d> texCoordList;       // List of texture coordinate
	vector<Vector3d> normalList;         // List of texture coordinate

	vector<_Polygon*> polyList;          // Faces of polygonal mesh

	Box bbox;                            // Bounding box of polygonal mesh
	Vector3d centroid;                   // Centroid of polygonal mesh
	double volume;
	Vector3d lowestPt;


public:
	PolyMesh();
	~PolyMesh();
	void ClearMesh();
	void PrintMesh();

	// Mesh Operations	
	void ComputeBBox();
	Box ComputeTextureBBox();
	void NormalizeMesh();

	// Volume and Centroid
	void ComputeCentroid();
	Vector3d ComputeCentroid(vector<Triangle*> triList);
	void ComputeVolume();
	double ComputeVolume(vector<Triangle*> triList);

	void ComputeLowestPt();
	Vector3d ComputeExtremeVertex(Vector3d rayDir);

	// Transform Mesh
	void RotateMesh(Vector3d rotCenter, Vector3d rotAxis, double rotAngle);

	// Read/Save OBJ File
	bool ReadOBJModel(const char *fileName);
	void WriteOBJModel(const char *objFileName);
	void UpdateVertices();

	// Draw Mesh
	void DrawMesh();
	void DrawMesh_Texture();
	void DrawMesh_Wire(double width, Vector3d color);
	void DrawMesh_Vertex(double size, Vector3d color);
	void DrawMesh_Normal(Vector3d color);
	void DrawMesh_Debug();
    void DrawMesh_Footprint();

	// Draw Mesh Property
	void DrawMeshBBox(double width, Vector3d color);
	void DrawMeshCentroid(double size, Vector3d color);

	// Draw Mesh to Buffer
	GLubyte* DrawMeshOffline(double viewW, double viewH, GLint iWidth, GLint iHeight);
	vector<Vector3i> Rasterize2DLine(Vector3i staPixelPos, Vector3i endPixelPos);
	bool GetPixelLocation(Vector3d crossPt, double viewW, double viewH, GLint iWidth, GLint iHeight, Vector3i &pixelPos);
};


#endif