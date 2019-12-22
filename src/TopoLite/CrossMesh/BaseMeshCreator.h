///////////////////////////////////////////////////////////////
//
// Remesh_Para.h
//
//  Convert Parameterized Mesh into Cross Mesh
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _REMESH_PARA_H
#define _REMESH_PARA_H

#include <vector>
#include "Utility/vec.h"
#include "QuadTree.h"
#include "Mesh/HEdgeMesh.h"

typedef shared_ptr<HEdgeMesh> pHEdgeMesh;

using namespace std;


class PolyMesh;
class CrossMesh;

typedef shared_ptr<PolyMesh> pPolyMesh;
typedef shared_ptr<CrossMesh> pCrossMesh;

/*!
 * \brief Mapping the 2D pattern into the 3D surface
 * \return the base polygonal mesh
 */
class BaseMeshCreator: public TopoObject
{
public:

    weak_ptr<QuadTree> quadTree;

	vector<int> map_vertex2D_3D;

	vector<int> map_vertex3D_2D;

	vector<int> map_cross2D_3D;

	vector<int> map_cross3D_2D;

	weak_ptr<PolyMesh> polyMesh;

	weak_ptr<CrossMesh> pattern2D;

	const float viewSize = 2.0;   // Note: since the 2D pattern mesh has been normalized into [-1.0, 1.0]

public:

    BaseMeshCreator(    shared_ptr<QuadTree> _quadTree,
                        shared_ptr<PolyMesh> _polyMesh,
                        shared_ptr<CrossMesh> _pattern2D,
                        shared_ptr<InputVarList> var);

    BaseMeshCreator(shared_ptr<InputVarList> var);

    ~BaseMeshCreator();

	// Compute Lifted 3D Mesh 

	/*!
	 * \brief: main function for mapping the 2D pattern into 3D surface
	 * \param[in] polyMesh: input 3D guide surface
	 * \param[in] pattern2D: input 2D pattern
	 * \param[in] inverTextureMat: User interaction of the 2D pattern (Rotation, Translation and Scale)
	 * \param[out] baseMesh2D: output the base 2D mesh
	 * \param[out] baseMesh: output the base mesh
	 * \note: it requires polyMesh with texture.
	 */

	void Pattern2CrossMesh(double *inverTextureMat,
                           shared_ptr<PolyMesh> &baseMesh2D,
                           shared_ptr<CrossMesh> &crossMesh);


    void PolyMesh2CrossMesh(pPolyMesh polyMesh,
                            pCrossMesh &crossMesh);

    void InitCrossMesh(     pPolyMesh polyMesh,
                            pCrossMesh &crossMesh);

	void ComputeInsideCross(double *inverTextureMat,
							shared_ptr<PolyMesh> &baseMesh2D,
							shared_ptr<CrossMesh> &crossMesh);

	void ComputeBoundaryCross(double *inverTextureMat,
							  shared_ptr<PolyMesh> &baseMesh2D,
							  shared_ptr<CrossMesh> &crossMesh);

    void ComputePracticalBoundary(shared_ptr<CrossMesh> &crossMesh);

	void ComputeCrossNeighbors(pHEdgeMesh hedgeMesh, pCrossMesh crossMesh);

	void RemoveDanglingCross(shared_ptr<CrossMesh> crossMesh);

	bool ComputeBoundaryVertex(double inverTextureMat[16], Vector3f sta2D, Vector3f end2D, Vector3f &pos2D, Vector3f &pos3D);
	/*!
	 * \brief: scale the 2D pattern position into UV space
	 */
	Vector3f GetTextureCoord(Vector3f point, float viewSize);

	/*!
	 * \brief: project the 2D ptTexCoord into the Surface
	 * \return: the 3D position of 2D pattern vertices
	 */
	bool ComputeSurfaceCoord(pPolyMesh polyMesh, Vector3f ptTexCoord, Vector3f &ptSurfCoord);


//
//	// Remove Dangling Polygon
//	void RemoveDanglingCross(pCrossMesh crossMesh);
};

#endif


