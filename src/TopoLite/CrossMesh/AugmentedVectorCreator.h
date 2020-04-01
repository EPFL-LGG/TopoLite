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

#include "Mesh/PolyMesh.h"
#include "Mesh/CrossMesh.h"
#include "Utility/TopoObject.h"

#include <tbb/tbb.h>
#include <utility>
#include <queue>
#include <unordered_map>

using namespace std;



/*!
 * \brief: Create CrossMesh by Input the polygonal mesh and tiltAngle
 */

template<typename Scalar>
class AugmentedVectorCreator : public TopoObject
{
public:

    typedef shared_ptr<CrossMesh<Scalar>> pCrossMesh;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

    typedef shared_ptr<Cross<Scalar>> pCross;

    typedef weak_ptr<Cross<Scalar>> wpCross;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;

public:

    explicit AugmentedVectorCreator(shared_ptr<InputVarList> var):TopoObject(std::move(var)){}

    ~AugmentedVectorCreator();

public:

    /*!
     * \brief Create crossMesh by setting alterative tiltAngle in polyMesh
     * \note This is the main function
     */
    void createAugmentedVector(Scalar tiltAngle, pCrossMesh crossMesh);

public:

    /*!
     * \brief Set initial tilt angle for each cross.
     * \note: the initial tilt angle is 0.
     */
    static void InitMeshTiltNormals(pCrossMesh crossMesh);

    /*!
     * \brief Distribute the sign of each tilt angle
    * \todo Only consider one possible distribution of sign for each edge. Other distribution may exist and can improve the structural stability
     */
    void InitMeshTiltNormalsResolveConflicts(pCrossMesh crossMesh, Scalar tiltAngle);

    void UpdateMeshTiltNormals(pCrossMesh crossMesh, Scalar tiltAngle);

    bool UpdateMeshTiltRange(pCrossMesh crossMesh);

};

#include "AugmentedVectorCreator.cpp"
#endif


