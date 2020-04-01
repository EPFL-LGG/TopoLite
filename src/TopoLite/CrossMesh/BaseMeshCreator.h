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

    typedef std::unordered_map<Cross<Scalar> *, int> mapCrossInt;

public:

    //map the vertices of pattern2D onto polyMesh
    //null when the vertices in outside of polyMesh's texture space
    vector<pVertex> pattern2D_vertices_on_polyMesh;

    //a polygon of pattern2D locates on the boundary of polyMesh's texture space
    //null if not
    tbb::concurrent_vector<pPolygon> boundary_pattern2D;

    wpPolyMeshAABB polyMesh;

    wpCrossMesh pattern2D;

	const float viewSize = 2.0;   // Note: since the 2D pattern mesh has been normalized into [-1.0, 1.0]

public:

    BaseMeshCreator(pPolyMeshAABB _polyMesh,
                    pCrossMesh _pattern2D,
                    shared_ptr<InputVarList> varList);

    ~BaseMeshCreator();



public:

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
	                          pCrossMesh &crossMesh,
	                          bool previewMode = false);

public:

	void computeInternalCross(Matrix4 textureMat,
	                          pPolyMesh &baseMesh2D,
	                          pCrossMesh &crossMesh);

	void computeBoundaryCross(Matrix4 textureMat,
                              pPolyMesh &baseMesh2D,
                              pCrossMesh &crossMesh);

	void removeSmallCrosses(pCrossMesh crossMesh);

    void recomputeBoundary(pCrossMesh crossMesh);

	void removeDanglingCross(pCrossMesh crossMesh);

public:
	/*!
	 * \brief: scale the 2D pattern position into UV space
	 */
    Vector2 getTextureCoord(Vector2 point, Matrix4 textureMat);

	/*!
	 * \brief: project the 2D ptTexCoord into the Surface
	 * \return: the 3D position of 2D pattern vertices
	 */
	bool mapTexPointBackToSurface(Vector2 ptTexCoord, Vector3 &ptSurfCoord);

	void splitIntoConsecutivePolygons(const vector<Line<Scalar>> &line, const vector<bool>& inside, tbb::concurrent_vector<pPolygon> &polyList);

    Matrix4 computeTextureMat(const pPolyMesh &referenceSurface, Matrix4 interactMat);
};

#include "BaseMeshCreator.cpp"

#endif


