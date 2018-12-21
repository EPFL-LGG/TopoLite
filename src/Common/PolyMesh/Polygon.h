///////////////////////////////////////////////////////////////
//
// PolyMesh/Polygon.h
//
//   3D Polygon (vertices may or may not be co-planar)
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _POLYGON_H
#define _POLYGON_H

#include "../HelpStruct.h"

using namespace std;


class Triangle;


class _Vertex
{
public:
	Vector3d pos;
	Vector2d texCoord;

	//Vector3d normal;
	//Vector3d color;


public:
	_Vertex()
	{

	};

	_Vertex(Vector3d _pos)
	{
		pos   = _pos;
	};

	_Vertex(Vector3d _pos, Vector2d _texCoord)
	{
		pos = _pos;

		texCoord = _texCoord;
	};
};


class _Polygon                           // Note: we do not name the class as Polygon to avoid confusion with Polygon defined in windows.h
{
public:
	vector<_Vertex> vers;                // Vertices (position, normal, texture coordinate etc.)
	vector<int> verIDs;                  // Vertex IDs (only for position)

	Vector3d normal;                     // Normal vector
	Vector3d center;                     // Center point


private: 
	int dist;                            // Discrete distance to a root polygon (e.g., BFS search)

	int polyType;                        // Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation


public:
	_Polygon();
	~_Polygon();
	_Polygon & operator=(const _Polygon &poly);
	bool IsEqual(_Polygon *poly); 
	void Print();
	void SetVertices(vector<Vector3d> _vers);

	// Compute Properties
	Vector3d ComputeCenter();
	Vector3d ComputeNormal();
	double ComputeArea();
	double ComputeAverageEdge();
	void ReverseVertices();

	// Access Operations
	void SetDistance(int _dist);
	int GetDistance();
	void SetPolyType(double _polyType);
	int GetPolyType();
	vector<Vector3d> GetVertices();

	// Polygon Operations
	vector<Triangle*> Convert2Triangles();
	int GetVertexIndexInList(Vector3d tagtVerPos);

	// Transforms
	void Translate(Vector3d transVec);
	vector<Vector3d> ProjectPolygonTo2D(double projMat[]);
	void ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[]);
	void ComputeFrame(Vector3d &x_axis, Vector3d &y_axis, Vector3d &origin);
	vector<Vector3i> ProjectToNormalPlane(Vector3d x_axis, Vector3d y_axis, Vector3d origin, double Scale);

	// Rendering
	void DrawPolygon(double width, Vector3d color);
};

#endif