///////////////////////////////////////////////////////////////
//
// Model.h
//
//  3D Object Model
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _MODEL_H
#define _MODEL_H

#include "TopoLite/Utility/TopoObject.h"
#include "TopoLite/Mesh/CrossMesh.h"
#include "TopoLite/Mesh/PolyMesh_AABBTree.h"
#include "BaseMeshCreator.h"
#include "AugmentedVectorCreator.h"
#include "Eigen/Dense"
#include <vector>
#include "igl/lscm.h"

/*!
 * CrossMesh = BaseMesh + AugmentedVector
 */
template <typename Scalar>
class CrossMeshCreator : public TopoObject
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

    typedef shared_ptr<VTex<Scalar>> pVTex;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef Matrix<Scalar, 4, 4> Matrix4;

    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

    typedef std::unordered_map<Cross<Scalar> *, int> mapCrossInt;

public:

	pPolyMeshAABB referenceSurface;      //<! Polygonal mesh of the reference surface model

	pCrossMesh crossMesh;           //<! Cross mesh

    pCrossMesh pattern2D;            //<! 2D tessellation mesh

public:

	int default_patternRadius;
	int default_patternID;

public:
	CrossMeshCreator(const CrossMeshCreator &_model);
    CrossMeshCreator(shared_ptr<InputVarList> var);
	~CrossMeshCreator();

	void clear();

	/*!
	 * \brief: load a .obj model from file
	 * \param objFileName: .obj file path
	 * \param texturedModel: True if use the texture
	 */
	bool loadSurface(const char *objFileName);

	bool setReferenceSurface(pPolyMesh surface);

    bool setPatternMesh(pPolyMesh surface);

	bool setCrossMesh(pPolyMesh surface, vector<bool> &atBoundary);

	/*!
	 * \brief: main function to create base CrossMesh
	 * \param texturedModel: True if the geometry of cross mesh is unknown. False if the polymesh just need to be assigned the tilt angle
	 * \param tiltAngle: Initial tilt angle
	 * \param patternID: Specific the pattern to create the geometry of cross mesh
	 * \param patternRadius: how many polygons the pattern has (the size the of the pattern)
	 * \param interactMatrix: User interaction of the pattern (scale, rotate, translate)
	 */
    bool createCrossMesh(   bool previewMode,
                            Matrix4 textureMat);

    bool updateTiltRange();

private:

	void recomputeTexture();
};

#include "CrossMeshCreator.cpp"

#endif

