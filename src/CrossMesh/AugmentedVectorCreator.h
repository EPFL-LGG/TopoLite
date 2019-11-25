///////////////////////////////////////////////////////////////
//
// Remesh_Own.h
//
//  Convert Polygonal Mesh into Cross Mesh
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 02/Feb/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _REMESH_OWN_H
#define _REMESH_OWN_H

#include "Utility/vec.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/HEdgeMesh.h"
#include "Mesh/CrossMesh.h"
#include "Utility/TopoObject.h"

#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>
#include <libShapeOp/src/Common.h>
#include <igl/AABB.h>
#include <vector>

using namespace std;

using pCrossMesh = shared_ptr<CrossMesh>;
using pHEdgeMesh = shared_ptr<HEdgeMesh>;
using pPolyMesh = shared_ptr<PolyMesh>;

/*!
 * \brief: Create CrossMesh by Input the polygonal mesh and tiltAngle
 */
class AugmentedVectorCreator : public TopoObject
{
public:
	weak_ptr<igl::AABB<Eigen::MatrixXd,3>> aabbTree;

	std::shared_ptr<Eigen::MatrixXd> aabbV;

	std::shared_ptr<Eigen::MatrixXi> aabbF;

public:

	AugmentedVectorCreator();

    AugmentedVectorCreator(shared_ptr<gluiVarList> var):TopoObject(var){AugmentedVectorCreator();}

	~AugmentedVectorCreator();

public:

	/*!
	 * \brief Create crossMesh by setting alterative tiltAngle in polyMesh
	 * \note This is the main function
	 */
	void CreateCrossMesh_Own  (pPolyMesh polyMesh, float tiltAngle, pCrossMesh &crossMesh);

	void CreateCrossMesh_Own  (float tiltAngle, pCrossMesh &crossMesh);

public:

	/*!
	 * \brief Initialize crossMesh's Geomtery.
	 * \note The edges' tile angles are not setted.
	 */
	void InitCrossMesh        (pPolyMesh polyMesh, pCrossMesh &crossMesh);

	/*!
	 * \brief: Initialize crossMesh's Connectivity.\n
	 * Give each cross its neighboring information.
	 */
	void ComputeCrossNeighbors(pHEdgeMesh hedgeMesh, pCrossMesh crossMesh);

	/*!
	 * \brief Set initial tilt angle for each cross.
	 * \note: the initial tilt angle is 0.
	 */
	void InitMeshTiltNormals(pCrossMesh crossMesh);

	/*!
 	* \brief Distribute the sign of each tilt angle
	* \todo Only consider one possible distribution of sign for each edge. Other distribution may exist and can improve the structural stability
 	*/
	void UpdateMeshTiltNormals(pCrossMesh crossMesh, float tiltAngle);

	bool UpdateMeshTiltRange(pCrossMesh crossMesh);

	void setAABBTree(shared_ptr<igl::AABB<Eigen::MatrixXd,3>> _aabbTree,
	std::shared_ptr<Eigen::MatrixXd> V,
			std::shared_ptr<Eigen::MatrixXi> F);

	void OptimizeBaseMesh(shared_ptr<CrossMesh> crossMesh);

	void ComputePracticalBoundary(shared_ptr<CrossMesh> &crossMesh);
};

#endif


