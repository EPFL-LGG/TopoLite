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


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//
#include "CrossMeshCreator.h"

template <typename Scalar>
CrossMeshCreator<Scalar>::CrossMeshCreator(shared_ptr<InputVarList> var) :TopoObject(var)
{
	referenceSurface = nullptr;

	pattern2D = nullptr;

	crossMesh = nullptr;

	default_patternRadius = -1;

	default_patternID = -1;
}
template <typename Scalar>
void CrossMeshCreator<Scalar>::clear()
{
    referenceSurface.reset();
    pattern2D.reset();
    crossMesh.reset();

    default_patternRadius = -1;
    default_patternID = -1;
}

template <typename Scalar>
CrossMeshCreator<Scalar>::CrossMeshCreator(const CrossMeshCreator &_model) : TopoObject(_model)
{
    clear();

    default_patternRadius = _model.default_patternRadius;
    default_patternID = _model.default_patternID;

    if(_model.pattern2D) pattern2D = make_shared<CrossMesh<Scalar>>(*_model.pattern2D);
    if(_model.referenceSurface) referenceSurface = make_shared<PolyMesh_AABBTree<Scalar>>(*_model.referenceSurface);
    if(_model.crossMesh) crossMesh = make_shared<CrossMesh<Scalar>>(*_model.crossMesh);
}

template <typename Scalar>
CrossMeshCreator<Scalar>::~CrossMeshCreator()
{

}

//**************************************************************************************//
//                                   Load Surface Model
//**************************************************************************************//

template <typename Scalar>
bool CrossMeshCreator<Scalar>::loadSurface(const char *objFileName)
{
	// 1. Read OBJ model
	shared_ptr<PolyMesh<Scalar>> surface;
    surface = make_shared<PolyMesh<Scalar>>(getVarList());
	bool texturedModel;

	if(surface->readOBJModel(objFileName, texturedModel, true))
	{
		// 2. Load a Texture model with parametrization.
		if (texturedModel)
		{
            referenceSurface = make_shared<PolyMesh_AABBTree<Scalar>>(*surface);
            getVarList()->set("texturedModel", true);
        }
		// 3. Load a Cross Mesh
		else{
            referenceSurface.reset();
            getVarList()->set("texturedModel", false);
            vector<bool> atBoundary;
            setCrossMesh(surface, atBoundary);
		}
		return true;
	}

	return false;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::setReferenceSurface(pPolyMesh _ref){

    if(_ref == nullptr) return false;
    if(!_ref->texturedModel)
    {
       referenceSurface = make_shared<PolyMesh_AABBTree<Scalar>>(*_ref);
       recomputeTexture();
    }
    else{
        referenceSurface = make_shared<PolyMesh_AABBTree<Scalar>>(*_ref);
        referenceSurface->buildTexTree();
    }

    if(referenceSurface)
    {
        getVarList()->set("texturedModel", true);
        default_patternRadius = -1;
        default_patternID = -1;
    }
    return true;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::setPatternMesh(pPolyMesh _pattern)
{
    if(_pattern == nullptr) return false;

    pattern2D = make_shared<CrossMesh<Scalar>>(*_pattern);
    
    //for user defined 2D pattern
    //we set patternRadius and patternID to be -1
    //to dinstinguish it from other predefined patterns.

    default_patternRadius = -1;
    default_patternID = -1;

    getVarList()->set("patternID", default_patternID);
    getVarList()->set("patternRadius", default_patternRadius);

    return true;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::setCrossMesh(pPolyMesh _cross, vector<bool> &atBoundary)
{
    if(_cross == nullptr) return false;

    getVarList()->set("texturedModel", false);
    crossMesh = make_shared<CrossMesh<Scalar>>(*_cross);
    BaseMeshCreator<Scalar> baseMeshCreator(getVarList());

    if(crossMesh->size() == atBoundary.size())
    {
        for(int id = 0; id < crossMesh->size(); id++)
        {
            pCross cross = crossMesh->cross(id);
            cross->atBoundary = atBoundary[cross->crossID];
        }
    }
    else{
        baseMeshCreator.recomputeBoundary(crossMesh);
    }

    float   tiltAngle       = getVarList()->template get<float>("tiltAngle");
    AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
    vectorCreator.createAugmentedVector(tiltAngle, crossMesh);

    return  true;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::createCrossMesh( bool previewMode, Matrix4 textureMat)
{
    float   tiltAngle       = getVarList()->template get<float>("tiltAngle");
    int   patternID       = getVarList()->template get<int>("patternID");
    float   patternRadius   = getVarList()->template get<int>("patternRadius");
    bool    texturedModel   = getVarList()->template get<int>("texturedModel");

    clock_t sta;
    if ( texturedModel )
    {
        //clear CrossMesh
        if (crossMesh != nullptr) crossMesh.reset();

        if(referenceSurface == nullptr) return false;

        if( default_patternID != patternID ||
            default_patternRadius != patternRadius ||
            pattern2D == nullptr)
        {
            sta = clock();
            PatternCreator<Scalar> patternCreator(getVarList());
            patternCreator.create2DPattern(PatternType(patternID), patternRadius, pattern2D);
            default_patternRadius = patternRadius;
            default_patternID = patternID;
            std::cout << "--2D PATTERN:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;
        }

        pPolyMesh baseMesh2D;

        sta = clock();
        BaseMeshCreator<Scalar> baseMeshCreator(referenceSurface, pattern2D, getVarList());
        baseMeshCreator.computeBaseCrossMesh(textureMat, baseMesh2D, crossMesh, previewMode);
        std::cout << "--Remesh Para:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

//        sta = clock();
//        BaseMeshOptimizer meshOptimizer(aabbTree, aabbV, aabbF, getVarList());
//        meshOptimizer.OptimizeBaseMesh(crossMesh);

        sta = clock();
        AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
        vectorCreator.createAugmentedVector(tiltAngle, crossMesh);
        std::cout << "--Remesh Own:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

        crossMesh->setBaseMesh2D(baseMesh2D);
    }
    else if(crossMesh){
        sta = clock();
        AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
        vectorCreator.UpdateMeshTiltNormals(crossMesh, tiltAngle);
        std::cout << "--UpdateMeshTiltNormals:\t" <<  (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;
    }

    return crossMesh != nullptr;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::updateTiltRange()
{
    //compute the tilt range
    if(crossMesh)
    {
        AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
        if(!vectorCreator.UpdateMeshTiltRange(crossMesh))
        {
            return false;
        }
        return true;
    }

    return false;
}

//**************************************************************************************//
//                                Utility Function
//**************************************************************************************//

template <typename Scalar>
void CrossMeshCreator<Scalar>::recomputeTexture() {
    if (referenceSurface) {
        MatrixX V;
        MatrixXi F;
        Eigen::VectorXi C;
        referenceSurface->convertPosToEigenMesh(V, F, C);

        // Fix two points on the boundary
        Eigen::VectorXi bnd, b(2, 1);
        igl::boundary_loop(F, bnd);
        b(0) = bnd(0);
        b(1) = bnd(round(bnd.size() / 2));
        MatrixX bc(2, 2);
        bc << 0, 0, 1, 0;

        MatrixX V_uv;
        // LSCM parametrization
        igl::lscm(V, F, b, bc, V_uv);

        referenceSurface->fromEigenMesh(V, V_uv, F);

        referenceSurface->buildTexTree();

        return;
    }
}

template<typename Scalar>
Matrix<Scalar, 4, 4> CrossMeshCreator<Scalar>::computeTextureMat_backwards_compatible(Matrix4 interactMat)
{
    //the following want to tranform the points in the surface texture space to the pattern space

    // Compute 2D bounding box of the parameterized surface mesh
    Box<Scalar> texBBox = referenceSurface->texBBox();

    //1) centralize the surface texture
    Matrix4 trans1 = Eigen::Matrix4d::Identity();
    trans1(0, 3) = -0.5*(texBBox.minPt.x() + texBBox.maxPt.x());
    trans1(1, 3) = -0.5*(texBBox.minPt.y() + texBBox.maxPt.y());
    trans1(2, 3) = 0;

    //2) scale 1) into [-0.5, -0.5]x [0.5, 0.5]
    Scalar scale_factor = getVarList()->template get<float>("textureScaleFactor");
    Scalar footScale = scale_factor / max(texBBox.maxPt.x() - texBBox.minPt.x(), texBBox.maxPt.y() - texBBox.minPt.y());
    Matrix4 scale = Eigen::Matrix4d::Identity();
    scale(0, 0) = footScale; scale(1, 1) = footScale;

    //3) tranform 2) by inveInteractMat
    Matrix4 inveInteractMat = interactMat.inverse();
    //Compatible issue with the old data
    //Reason: Since the scale of 2D pattern space is [-1, 1] while the scale of 2D texture space is [0, 1]
    inveInteractMat(0, 3) /= 2;
    inveInteractMat(1, 3) /= 2;
    inveInteractMat(2, 3) /= 2;

    //4) move the 3)'s center into [0.5, 0.5], so that it is within [0, 0] x[1, 1]
    Matrix4 trans2 = Eigen::Matrix4d::Identity();
    trans2(0, 3) = 0.5;
    trans2(1, 3) = 0.5;
    trans2(2, 3) = 0;

    Matrix4 textureMat = trans2 * inveInteractMat * scale * trans1;
    //5) inverse it because we want a transform form pattern space to surface texture space
    return textureMat.inverse();
}

template class CrossMeshCreator<double>;
