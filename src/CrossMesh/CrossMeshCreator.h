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

#include "Utility/vec.h"
#include "QuadTree.h"
#include "igl/AABB.h"
#include "Eigen/Dense"
#include "Utility/TopoObject.h"

#include <vector>

using pCrossMesh = shared_ptr<CrossMesh>;


/*!
 * CrossMesh = BaseMesh + AugmentedVector
 */
class CrossMeshCreator : public TopoObject
{
public:

	pPolyMesh referenceSurface;      //<! Polygonal mesh of the reference surface model

	pCrossMesh crossMesh;           //<! Cross mesh

    pCrossMesh pattern2D;            //<! 2D tessellation mesh

public:

    double textureNormalizeMat[16];          //<! Transform the texture coordinates to be within [0, 1]

    shared_ptr<QuadTree> quadTree;

	shared_ptr<igl::AABB<Eigen::MatrixXd,3>> aabbTree;

	shared_ptr<Eigen::MatrixXd> polyMeshV;

	shared_ptr<Eigen::MatrixXi> polyMeshF;

	int default_patternRadius;

	int default_patternID;

public:
	CrossMeshCreator(const CrossMeshCreator &_model);
    CrossMeshCreator();
    CrossMeshCreator(shared_ptr<gluiVarList> var) :TopoObject(var){CrossMeshCreator();};
	~CrossMeshCreator();

	void ClearModel();

	/*!
	 * \brief: load a .obj model from file
	 * \param objFileName: .obj file path
	 * \param texturedModel: True if use the texture
	 */
	bool LoadReferenceSurface(const char *objFileName);

	/*!
	 * \brief: main function to create base CrossMesh
	 * \param texturedModel: True if the geometry of cross mesh is unknown. False if the polymesh just need to be assigned the tilt angle
	 * \param tiltAngle: Initial tilt angle
	 * \param patternID: Specific the pattern to create the geometry of cross mesh
	 * \param patternRadius: how many polygons the pattern has (the size the of the pattern)
	 * \param interactMatrix: User interaction of the pattern (scale, rotate, translate)
	 */
	void CreateCrossMesh(   bool texturedModel,
	                        float tiltAngle,
	                        int patternID,
	                        int patternRadius,
	                        double interactMatrix[]);

public:

    /*!
    * \brief: For correct texture scale
    */
    void ComputeTextureNormalizeMatrix();

    /*!
     * \brief: Compute the correct texture
     */
    void ComputeTextureMatrix(double interactMatrix[], double textureMatrix[16]);


public:

	void OverwriteTexture();

	void CreateQuadTree();

	void CreateAABBTree();
};

#endif

