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

    AugmentedVectorCreator(shared_ptr<InputVarList> var):TopoObject(var){}

	~AugmentedVectorCreator();

public:

	/*!
	 * \brief Create crossMesh by setting alterative tiltAngle in polyMesh
	 * \note This is the main function
	 */
	void CreateAugmentedVector(float tiltAngle, pCrossMesh &crossMesh);

public:

	/*!
	 * \brief Set initial tilt angle for each cross.
	 * \note: the initial tilt angle is 0.
	 */
	void InitMeshTiltNormals(pCrossMesh crossMesh);

	/*!
 	* \brief Distribute the sign of each tilt angle
	* \todo Only consider one possible distribution of sign for each edge. Other distribution may exist and can improve the structural stability
 	*/
	void InitMeshTiltNormalsResolveConflicts(pCrossMesh crossMesh, float tiltAngle);


	void UpdateMeshTiltNormals(pCrossMesh crossMesh, float tiltAngle);

	bool UpdateMeshTiltRange(pCrossMesh crossMesh);

};

#endif


