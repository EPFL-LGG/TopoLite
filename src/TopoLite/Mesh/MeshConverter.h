///////////////////////////////////////////////////////////////
//
// MeshConverter.h
//
//   Convert in between different mesh representations:
//   1) Triangular mesh
//   2) Polygonal mesh 
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 21/July/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_CONVERTER_H
#define _MESH_CONVERTER_H

#include "Utility/vec.h"
#include "Polygon.h"
#include "Utility/TopoObject.h"
#include "IO/InputVar.h"
#include "Mesh/PolyMesh.h"

#include <vector>
#include <functional>
#include <Eigen/Dense>
#include <igl/remove_duplicate_vertices.h>
#include <clipper.hpp>

using namespace std;
using pPolyMesh = shared_ptr<PolyMesh>;
using pPolygon = shared_ptr<_Polygon>;
using pTriangle = shared_ptr<Triangle>;

struct PolyMeshRhino{
    vector<Vector3f> vertices;
    vector<Vector3i> faces;
    vector<vector<int>> verticesGroups;
    vector<vector<int>> facesGroups;
};

class MeshConverter: public TopoObject
{

public:
    MeshConverter(){}
    MeshConverter(std::shared_ptr<InputVarList> var) : TopoObject(var){}
	~MeshConverter(){}

public:
    //generate texture (surface -> [0,1]x[0,1] square)
    //by using LSCM method from libigl

    void generateTexture(const PolyMesh *polyMesh, shared_ptr<PolyMesh> &out);

public:
	// Triangulation

	void Convert2TriMesh(const PolyMesh *polyMesh, vector<pTriangle> &out);

	void Convert2TriMesh(const PolyMesh *polyMesh, shared_ptr<PolyMesh> &out);

public:
    // Convert to Mesh

    void InitPolyMesh(const vector<Vector3f>& inVerList, const vector<Vector3i>& inTriList, pPolyMesh &out);

    void Convert2PolyMesh(const vector<Vector3f> &inVerList, const vector<Vector3i> &inTriList, pPolyMesh &out);

    void InitPolyMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, pPolyMesh &out);

    void Convert2PolyMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, pPolyMesh &out);

	void Convert2PolyMesh(pPolyMesh polyMesh);

public:
    // Convert to EigenMesh

    void Convert2EigenMesh(const vector<Vector3f> &inVerList, const vector<Vector3i> &inTriList, Eigen::MatrixXd &V, Eigen::MatrixXi &F);

    void Convert2EigenMesh(const PolyMesh *polyMesh, Eigen::MatrixXd &V, Eigen::MatrixXi &F);

    void Convert2EigenMesh( const PolyMesh *polyMesh,
                            PolyMeshRhino *rhinoMesh);
public:

    //Clipper Countour <-> Mesh

    void EigenMesh2ClipperPath(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, ClipperLib::Paths &contour);

//    void ClipperPathExtrusion(ClipperLib::Paths &contour, double height, shared_ptr<PolyMesh> &out);


private:
    //private function

    bool IsCoplanar(pPolygon triA, pPolygon triB);

	void UpdatePolyList(pPolygon polyA, int polyAID, pPolygon polyB, int polyBID, const vector<Vector3f> vertexList, vector<pPolygon> &polyList);

	void MergePolygons(pPolygon polyA, pPolygon polyB, const vector<Vector3f> vertexList, pPolygon &out);
};


#endif