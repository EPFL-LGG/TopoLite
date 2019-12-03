///////////////////////////////////////////////////////////////
//
// Mesh/Polygon.h
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

#include "Utility/vec.h"
#include "Utility/HelpStruct.h"
#include <vector>

using namespace std;
using pTriangle = shared_ptr<Triangle>;

class _Vertex
{
public:

	Vector3f pos;				//!< vertex's postion (X,Y,Z)

	Vector2f texCoord;			//!< vertex's texcoord (U,V) for mapping the pattern

public:
	_Vertex()
	{

	};

	_Vertex(Vector3f _pos)
	{
		pos   = _pos;
	};

	_Vertex(Vector3f _pos, Vector2f _texCoord)
	{
		pos = _pos;

		texCoord = _texCoord;
	};
};

/*!
 * @brief: Polygon class
 * @note: we do not name the class as Polygon to avoid confusion with Polygon defined in windows.h
 */
class _Polygon
{
public:

    //storage
	vector<_Vertex> vers;                //!< Vertices   (position, texture coordinate.)
	vector<int> verIDs;                  //!< Vertex IDs (only for position)
	vector<int> texIDs;					 //!< texture ID

    //computed
	Vector3f normal;                     //!< Normal vector
	Vector3f center;                     //!< Center point

	//auxiliary
    int dist;                            //!< Discrete distance to a root polygon (e.g., BFS search)
    int polyType;                        //!< Polygon type (number of edges, shape, orientation); note: this variable is used for generating 2D tiling tessellation

public:
    //Basic Operations

    _Polygon();
    _Polygon(const _Polygon &poly);
    _Polygon & operator=(const _Polygon &poly);
	~_Polygon();

    void SetVertices(vector<Vector3f> _vers);
    void ReverseVertices();
    size_t push_back(Vector3f pt);

    bool IsEqual(_Polygon* poly);
    void Print();

public:
    // Compute Properties
	Vector3f ComputeCenter();
	Vector3f ComputeNormal();
	Vector3f PlaneNormalFit();
	float ComputeArea();
	float ComputeAverageEdge();

public:
	// Access Operations
	void SetDistance(int _dist);
	int GetDistance();
	void SetPolyType(float _polyType);
	int GetPolyType();
	vector<Vector3f> GetVertices();
	vector<Vector2f> GetVerticesTex();

	// Polygon Operations
	void Convert2Triangles(vector<pTriangle> &out);
	int GetVertexIndexInList(Vector3f tagtVerPos);

	// Transforms
	void Translate(Vector3f transVec);
	vector<Vector3f> ProjectPolygonTo2D(double projMat[]);
	void ComputeProjectMatrixTo2D(double projMat[], double invsProjMat[]);
	void ComputeFrame(Vector3f &x_axis, Vector3f &y_axis, Vector3f &origin);
	vector<Vector3i> ProjectToNormalPlane(Vector3f x_axis, Vector3f y_axis, Vector3f origin, float Scale);
};

#endif