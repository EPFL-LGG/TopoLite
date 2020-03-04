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
#include <unordered_map>
#include <tbb/tbb.h>

#include "TopoLite/Mesh/PolyMesh_AABBTree.h"
#include "Mesh/CrossMesh.h"

using namespace std;

/*!
 * \brief Mapping the 2D pattern into the 3D surface
 * \return the base polygonal mesh
 */

template<typename Scalar>
class BaseMeshCreator: public TopoObject
{
public:

    typedef shared_ptr<PolyMesh_AABBTree<Scalar>> pPolyMeshAABB;

    typedef weak_ptr<PolyMesh_AABBTree<Scalar>> wpPolyMeshAABB;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

    typedef weak_ptr<PolyMesh<Scalar>> wpPolyMesh;

    typedef shared_ptr<CrossMesh<Scalar>> pCrossMesh;

    typedef weak_ptr<CrossMesh<Scalar>> wpCrossMesh;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef weak_ptr<_Polygon<Scalar>> wpPolygon;

    typedef shared_ptr<Cross<Scalar>> pCross;

    typedef weak_ptr<Cross<Scalar>> wpCross;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef Matrix<Scalar, 4, 4> Matrix4;

public:

	vector<int> map_vertex2D_3D;

	vector<int> map_vertex3D_2D;

	vector<int> map_cross2D_3D;

	vector<int> map_cross3D_2D;

    wpPolyMeshAABB polyMesh;

    wpCrossMesh pattern2D;

	const float viewSize = 2.0;   // Note: since the 2D pattern mesh has been normalized into [-1.0, 1.0]

public:

    BaseMeshCreator(pPolyMeshAABB _polyMesh,
                    pCrossMesh _pattern2D);

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
	void computeBaseCrossMesh(Matrix<Scalar, 4, 4> interactMat,
	                          pPolyMesh &baseMesh2D,
	                          pCrossMesh &crossMesh);

    void InitCrossMesh(     pPolyMesh polyMesh,
                            pCrossMesh &crossMesh);

	void computeInternalCross(Matrix4 textureMat,
	                          pPolyMesh &baseMesh2D,
	                          pCrossMesh &crossMesh);

	void ComputeBoundaryCross(Matrix4 textureMat,
							  pPolyMesh &baseMesh2D,
							  pCrossMesh &crossMesh);

    void ComputePracticalBoundary(pCrossMesh &crossMesh);

//	void ComputeCrossNeighbors(pHEdgeMesh hedgeMesh, pCrossMesh crossMesh);

	void RemoveDanglingCross(pCrossMesh crossMesh);

	bool ComputeBoundaryVertex(double inverTextureMat[16], Vector3 sta2D, Vector3 end2D, Vector3 &pos2D, Vector3 &pos3D);


    Matrix4 computeTextureMat(const pPolyMesh &referenceSurface, Matrix4 interactMat);

private:
	/*!
	 * \brief: scale the 2D pattern position into UV space
	 */
    Vector2 GetTextureCoord(Vector2 point, Matrix4 textureMat);

	/*!
	 * \brief: project the 2D ptTexCoord into the Surface
	 * \return: the 3D position of 2D pattern vertices
	 */
	bool mapTexPointBackToSurface(Vector2 ptTexCoord, Vector3 &ptSurfCoord);


//
//	// Remove Dangling Polygon
//	void RemoveDanglingCross(pCrossMesh crossMesh);
};

#include "BaseMeshCreator.cpp"

#endif


