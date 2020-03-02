///////////////////////////////////////////////////////////////
//
// MeshCreator.h
//
//   Create Polygonal Meshes
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _MESH_CREATOR_H
#define _MESH_CREATOR_H

#include <vector>
#include "Mesh/CrossMesh.h"

template<typename Scalar>
class PatternCreator : public TopoObject
{
public:
    typedef shared_ptr<VPoint<Scalar>> pVertex;

    struct pVertex_compare
    {
        bool operator()(const pVertex &A, const pVertex &B) const
        {
            double eps = FLOAT_ERROR_LARGE;

            if (A.pos[0] - B.pos[0] < -eps)
                return true;
            if (A.pos[0] - B.pos[0] > eps)
                return false;

            if (A.pos[1] - B.pos[1] < -eps)
                return true;
            if (A.pos[1] - B.pos[1] > eps)
                return false;

            if (A.pos[2] - B.pos[2] < -eps)
                return true;
            if (A.pos[2] - B.pos[2] > eps)
                return false;

            return false;
        }
    };

    typedef std::set<pVertex, pVertex_compare> setVertex;

public:
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef shared_ptr<CrossMesh<Scalar>> pCrossMesh;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;


public:
	PatternCreator();
	PatternCreator(shared_ptr<InputVarList> var):TopoObject(var){}
	~PatternCreator();

public:

	// Create Mesh (2D Regular Pattern)
	void create2DPattern(int patternID,
	                     int patternRadius,
	                     pCrossMesh &out);

    // Create Polygons
    void createPolygonRoot(int edgeNum, Scalar edgeLen, pPolygon &out);

    // Compute Neighbors
    void computeNeighbors(int crosstype, pPolygon poly, vector<pPolygon> &out);

    void addToPolyMesh(pPolygon poly, pPolyMesh mesh, setVertex &vertices_set);

    bool checkPolygonExistance(pPolygon poly, const setVertex &vertices_set);

    Vector3 rotateVector(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle, Vector3 tagtPt);

public:

    // Create Mesh from Multiple Meshes
    void CreateMesh_Merge(vector<pPolyMesh> polyMeshes, pPolyMesh &out);

	void CreateMesh_2DPattern(vector<_Polygon<Scalar>> &root_polys,
                              Vector3 DX,
                              Vector3 DY,
                              int Nx,
                              int Ny,
                              pCrossMesh &out);


	void CreateMesh_2DPattern_HexagonPattern2(int patternRadius, pCrossMesh &out);
    void CreateMesh_2DPattern_HexagonPattern3(int patternRadius, pCrossMesh &out);
    void CreateMesh_2DPattern_PentagonPattern3(int patternRadius, pCrossMesh &out);
	int GetPolygonIndexInList(pPolygon tagtPoly, vector<pPolygon> polyList);

public:



	void ComputeNeighbor_Square(pPolygon poly, pPolygon &out);
	void ComputeNeighbor_Hexagon(pPolygon poly, pPolygon &out);
	void ComputeNeighbor_Octagon_Square(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Hexagon_Rhombus(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Dodecagon(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Dodecagon_Hexagon_Square(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Square_Rhombus(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Cross(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Snow(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Pentagon_Mirror(pPolygon poly, int edgeID, pPolygon &out);
	void ComputeNeighbor_Rhombus(pPolygon poly, int edgeID, pPolygon &out);

    void ComputeNeighbor_Octagon_Square_Colinear(pPolygon poly, int edgeID, pPolygon &out);

    Matrix<Scalar, 3, 1> ComputeTileTranslation(pPolygon neighbor, Vector3 tagtStaPt, Vector3 tagtEndPt);
    Matrix<Scalar, 3, 1> ComputeTileTranslation_OCTAGON_SQUARE_COLINEAR(pPolygon poly, int edgeID);


	void CreatePolygon_Square(pPolygon &out, Scalar edgeLen, int polyType = POLY_SQUARE_THETA_45);
	void CreatePolygon_Hexagon(pPolygon &out, Scalar edgeLen, int polyType = POLY_HEXAGON_TYPE_0);
	void CreatePolygon_Octagon(pPolygon &out, Scalar edgeLen, int polyType);
	void CreatePolygon_Dodecagon(pPolygon &out, Scalar edgeLen);
	void CreatePolygon_Rhombus(pPolygon &out, Scalar edgeLen, int polyType = POLY_RHOMBUS_THETA_0);

	void CreatePolygon_Pentagon_Cross(pPolygon &out, Scalar edgeLen, int polyType = POLY_PENTAGON_CROSS_TYPE_0);
	void CreatePolygon_Pentagon_Snow(pPolygon &out, Scalar edgeLen, int polyType = POLY_PENTAGON_SNOW_TYPE_0);
    void CreatePolygon_Pentagon_Mirror(pPolygon &out, Scalar edgeLen, int polyType = POLY_PENTAGON_MIRROR_TYPE_0);

	// Create Mesh (a Single Fan)
	void CreateMesh_Fan(Vector3 edgeA, Vector3 edgeB, Vector3 edgeMid, pPolyMesh &out);
	void CreatePolygon_Fan(Vector3 verA, Vector3 verB, Vector3 verM, pPolygon &out);

public:
    //python
    vector<vector<double>> PyCreateMesh_2DPattern(int patternID, int patternRadius);

};

#include "PatternCreator.cpp"

#endif