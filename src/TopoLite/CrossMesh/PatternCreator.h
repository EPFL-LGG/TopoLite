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

enum PatternType{
    CROSS_SQUARE = 1,
    CROSS_RHOMBUS = 2,
    CROSS_SQUARE_RHOMBUS = 3,
    CROSS_HEXAGON = 4,
    CROSS_HEXAGON_RHOMBUS = 5,
    CROSS_DODECAGON_HEXAGON_QUAD = 6,
    CROSS_OCTAGON_SQUARE = 7,
    CROSS_OCTAGON_SQUARE_COLINEAR = 8,
    CROSS_DODECAGON = 9,
    CROSS_PENTAGON_CROSS = 10,
    CROSS_PENTAGON_SNOW = 11,
    CROSS_PENTAGON_MIRROR = 12
};

template<typename Scalar>
class PatternCreator : public TopoObject
{
public:
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef shared_ptr<CrossMesh<Scalar>> pCrossMesh;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;
    
public:
    
    struct pVertex_compare
    {
        bool operator()(const Vector3 &A, const Vector3 &B) const
        {
            double eps = FLOAT_ERROR_LARGE;

            if (A[0] - B[0] < -eps)
                return true;
            if (A[0] - B[0] > eps)
                return false;

            if (A[1] - B[1] < -eps)
                return true;
            if (A[1] - B[1] > eps)
                return false;

            if (A[2] - B[2] < -eps)
                return true;
            if (A[2] - B[2] > eps)
                return false;

            return false;
        }
    };

    typedef std::set<Vector3, pVertex_compare> setVertex;

public:
	PatternCreator();
	PatternCreator(shared_ptr<InputVarList> var):TopoObject(var){}
	~PatternCreator();

public:

	// Create Mesh (2D Regular Pattern)
	void create2DPattern(PatternType patternID,
	                     int patternRadius,
	                     pCrossMesh &out);

    // Create Polygons
    void createPolygonRoot(int edgeNum, Scalar edgeLen, pPolygon &out);

    // Compute Neighbors
    void computeNeighbors(PatternType crosstype, pPolygon poly, vector<pPolygon> &out);

    bool checkPolygonExistance(pPolygon poly, const setVertex &vertices_set);
    
    void addToVerticesSet(pPolygon poly, setVertex &vertices_set);

    Vector3 rotateVector(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle, Vector3 tagtPt);

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

public:
    //python
    vector<vector<double>> PyCreateMesh_2DPattern(int patternID, int patternRadius);

};

#include "PatternCreator.cpp"

#endif