///////////////////////////////////////////////////////////////
//
// Model.cpp
//
//  3D Object Model
//
// by Song Peng ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#include "time.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/vec.h"
#include "Utility/math3D.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/HEdgeMesh.h"
#include "PatternCreator.h"
#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "AugmentedVectorCreator.h"
#include "BaseMeshCreator.h"
#include "CrossMeshCreator.h"
#include "BaseMeshOptimizer.h"
#include "IO/InputVar.h"
#include "Mesh/MeshConverter.h"
#include <Eigen/Dense>

#ifndef CATCH2_UNITTEST

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

CrossMeshCreator::CrossMeshCreator(shared_ptr<InputVarList> var) :TopoObject(var)
{
    aabbTree = nullptr;

    quadTree = nullptr;

	referenceSurface = nullptr;

	pattern2D = nullptr;

	crossMesh = nullptr;

	default_patternRadius = -1;

	default_patternID = -1;
}

void CrossMeshCreator::ClearModel()
{
    aabbTree.reset();
    quadTree.reset();

    referenceSurface.reset();
    pattern2D.reset();
    crossMesh.reset();

    aabbV.reset();
    aabbF.reset();

    default_patternRadius = -1;
    default_patternID = -1;
}

CrossMeshCreator::CrossMeshCreator(const CrossMeshCreator &_model) : TopoObject(_model)
{
    ClearModel();

    default_patternRadius = _model.default_patternRadius;
    default_patternID = _model.default_patternID;

    if(_model.pattern2D) pattern2D = make_shared<CrossMesh>(*_model.pattern2D);
    if(_model.referenceSurface) referenceSurface = make_shared<PolyMesh>(*_model.referenceSurface);
    if(_model.crossMesh) crossMesh = make_shared<CrossMesh>(*_model.crossMesh);

    for(int id = 0;id < 16; id++)
    {
        textureNormalizeMat[id] = _model.textureNormalizeMat[id];
    }

    if(referenceSurface->texturedModel)
    {
        CreateQuadTree();
        CreateAABBTree();
    }
}

CrossMeshCreator::~CrossMeshCreator()
{

}

//**************************************************************************************//
//                                   Load Surface Model
//**************************************************************************************//

bool CrossMeshCreator::LoadReferenceSurface(const char *objFileName)
{
	// 1. Read OBJ model
	referenceSurface.reset();
	referenceSurface = make_shared<PolyMesh>(getVarList());
	bool texturedModel;

	if(referenceSurface->ReadOBJModel(objFileName, texturedModel, true))
	{

		getVarList()->set("texturedModel", texturedModel);
		referenceSurface->ComputeBBox();
		// 2. Textured model (with parameterization)
		if (texturedModel)
		{
            ComputeTextureNormalizeMatrix();
            CreateQuadTree();
            CreateAABBTree();
		}
		return true;
	}
	else{
		return false;
	}
}

bool CrossMeshCreator::LoadReferenceSurface(pPolyMesh surface, vector<bool> &atBoundary){
    referenceSurface.reset();
    referenceSurface = make_shared<PolyMesh>(*surface);
    getVarList()->set("texturedModel", false);
    referenceSurface->ComputeBBox();

    BaseMeshCreator baseMeshCreator(getVarList());
    baseMeshCreator.ComputeBaseMesh(referenceSurface, crossMesh);

    if(crossMesh->crossList.size() == atBoundary.size()){
        for(pCross cross: crossMesh->crossList){
            cross->atBoundary = atBoundary[cross->crossID];
        }
    }

    float   tiltAngle       = getVarList()->get<float>("tiltAngle");
    AugmentedVectorCreator vectorCreator(getVarList());
    vectorCreator.CreateAugmentedVector(tiltAngle, crossMesh);

    return  true;
}

bool CrossMeshCreator::CreateCrossMesh( bool previewMode,
                                        double interactMatrix[])
{
    float   tiltAngle       = getVarList()->get<float>("tiltAngle");
    float   patternID       = getVarList()->get<int>("patternID");
    float   patternRadius   = getVarList()->get<int>("patternRadius");
    bool    texturedModel   = getVarList()->get<int>("texturedModel");

    //clear CrossMesh
    if (crossMesh != nullptr) crossMesh.reset();

    if ( texturedModel )
    {
        if(referenceSurface == nullptr) return false;
        if(quadTree == nullptr) return false;
        if(aabbTree == nullptr) return false;

        clock_t sta;

        if( default_patternID != patternID ||
            default_patternRadius != patternRadius ||
            pattern2D == nullptr)
        {
            sta = clock();
            PatternCreator patternCreator(getVarList());
            patternCreator.CreateMesh_2DPattern(patternID, patternRadius, pattern2D);
            default_patternRadius = patternRadius;
            default_patternID = patternID;
            std::cout << "--2D PATTERN:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;
        }

        double textureMatrix[16];
        ComputeTextureMatrix(interactMatrix, textureMatrix);
        double inverTextureMat[16];
        memcpy(inverTextureMat, textureMatrix, sizeof(double) * 16);
        invertMat(inverTextureMat, 4);

        shared_ptr<PolyMesh> baseMesh2D;

        sta = clock();
        BaseMeshCreator baseMeshCreator(quadTree, referenceSurface, pattern2D, getVarList());
        baseMeshCreator.ComputeBaseMesh(inverTextureMat, baseMesh2D, crossMesh);
        std::cout << "--Remesh Para:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

        sta = clock();
        BaseMeshOptimizer meshOptimizer(aabbTree, aabbV, aabbF, getVarList());
        meshOptimizer.OptimizeBaseMesh(crossMesh);

        sta = clock();
        AugmentedVectorCreator vectorCreator(getVarList());
        vectorCreator.CreateAugmentedVector(tiltAngle, crossMesh);
        std::cout << "--Remesh Own:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

        crossMesh->SetBaseMesh2D(baseMesh2D);
    }
    else
    {
        BaseMeshCreator baseMeshCreator(getVarList());
        baseMeshCreator.ComputeBaseMesh(referenceSurface, crossMesh);

        AugmentedVectorCreator vectorCreator(getVarList());
        vectorCreator.CreateAugmentedVector(tiltAngle, crossMesh);
    }

    //compute the tilt range
    if(!previewMode && crossMesh)
    {
        AugmentedVectorCreator vectorCreator(getVarList());
        if(!vectorCreator.UpdateMeshTiltRange(crossMesh))
        {
            crossMesh.reset();
        }
    }

    return crossMesh != nullptr;
}

//**************************************************************************************//
//                                Texture Matrix
//**************************************************************************************//

void CrossMeshCreator::ComputeTextureNormalizeMatrix()
{
    // Compute 2D bounding box of the parameterized surface mesh
    Box texBBox = referenceSurface->ComputeTextureBBox();
    float scale_factor = getVarList()->get<float>("textureScaleFactor");
    float footScale = scale_factor / max(texBBox.maxPt.x - texBBox.minPt.x, texBBox.maxPt.y - texBBox.minPt.y);

    //Compute foot matrix that transform the 2D bbox into a unit 2D box [minPt(0,0), maxPt(1,1)]
    Eigen::Matrix4d mat;
    mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d trans1;
    trans1 = Eigen::Matrix4d::Identity();
    trans1(0, 3) = 0.5; trans1(1, 3) = 0.5; trans1(2, 3) = 0;
    Eigen::Matrix4d scale;
    scale = Eigen::Matrix4d::Identity();
    scale(0, 0) = footScale; scale(1, 1) = footScale;
    Eigen::Matrix4d trans2;
    trans2 = Eigen::Matrix4d::Identity();
    trans2(0, 3) = -0.5f*(texBBox.minPt.x + texBBox.maxPt.x); trans2(1, 3) = -0.5f*(texBBox.minPt.y + texBBox.maxPt.y); trans2(2, 3) = 0;
    mat = mat * trans1 * scale * trans2;
    for(int id = 0; id < mat.cols(); id++){
        for(int jd = 0; jd < mat.rows(); jd++)
            textureNormalizeMat[4 * id + jd] = mat(jd, id);
    }
}

void CrossMeshCreator::ComputeTextureMatrix(double interactMatrix[16], double textureMatrix[16])
{
    double inveInteractMat[16];
    memcpy(inveInteractMat, interactMatrix, sizeof(double) * 16);
    if (invertMat(inveInteractMat, 4) == 1)  printf("Inverse Matrix Error \n");

    inveInteractMat[12] = inveInteractMat[12] / 2.0f;   // Since the scale of 2D pattern space is [-1, 1] while the scale of 2D texture space is [0, 1]
    inveInteractMat[13] = inveInteractMat[13] / 2.0f;
    inveInteractMat[14] = inveInteractMat[14] / 2.0f;


    Eigen::Matrix4d mat;
    mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d trans1;
    trans1 = Eigen::Matrix4d::Identity();
    trans1(0, 3) = 0.5; trans1(1, 3) = 0.5; trans1(2, 3) = 0;
    Eigen::Matrix4d inveMat;
    for(int id = 0; id < mat.cols(); id++){
        for(int jd = 0; jd < mat.rows(); jd++)
            inveMat(jd, id) = inveInteractMat[4 * id + jd];
    }
    Eigen::Matrix4d trans2;
    trans2 = Eigen::Matrix4d::Identity();
    trans2(0, 3) = -0.5; trans2(1, 3) = -0.5f; trans2(2, 3) = 0;

    Eigen::Matrix4d footMat;
    for(int id = 0; id < mat.cols(); id++){
        for(int jd = 0; jd < mat.rows(); jd++)
            footMat(jd, id) = textureNormalizeMat[4 * id + jd];
    }

    mat = mat * trans1 * inveMat * trans2 * footMat;
    for(int id = 0; id < mat.cols(); id++)
    {
        for(int jd = 0; jd < mat.rows(); jd++)
            textureMatrix[4 * id + jd] = mat(jd, id);
    }
}

//**************************************************************************************//
//                                Auxiliary Tree
//**************************************************************************************//

void CrossMeshCreator::OverwriteTexture()
{
	if(referenceSurface)
	{
		shared_ptr<PolyMesh> texMesh;
		MeshConverter convert;
        convert.generateTexture(referenceSurface.get(), texMesh);
		referenceSurface = texMesh;
		getVarList()->set("texturedModel", true);
		texMesh->texturedModel = true;
		texMesh->ComputeBBox();
        CreateQuadTree();
	}
}

void CrossMeshCreator::CreateQuadTree()
{
	quadTree.reset();
	quadTree = make_shared<QuadTree>(referenceSurface, getVarList());
}

void CrossMeshCreator::CreateAABBTree()
{
    if(referenceSurface->vertexList.empty()){
        return;
    }

    int n = referenceSurface->vertexList.size();
    int m = referenceSurface->polyList.size();
    aabbV = make_shared<Eigen::MatrixXd>(n, 3);
    aabbF = make_shared<Eigen::MatrixXi>(m, 3);
    for(int id = 0; id < n; id++)
    {
        Vector3f ver = referenceSurface->vertexList[id];
        aabbV->row(id) = Eigen::RowVector3d(ver.x, ver.y, ver.z);
    }
    for(int id = 0; id < m; id++){
        shared_ptr<_Polygon> poly = referenceSurface->polyList[id];
        aabbF->row(id) = Eigen::RowVector3i(poly->verIDs[0], poly->verIDs[1], poly->verIDs[2]);
    }

    aabbTree.reset();
    aabbTree = make_shared<igl::AABB<Eigen::MatrixXd,3>>();
    aabbTree->init(*aabbV, *aabbF);

    return;
}

#else

#endif