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

    if(_model.pattern2D) pattern2D = make_shared<CrossMesh>(*_model.pattern2D);
    if(_model.referenceSurface) referenceSurface = make_shared<PolyMesh>(*_model.referenceSurface);
    if(_model.crossMesh) crossMesh = make_shared<CrossMesh>(*_model.crossMesh);
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
	pPolyMesh surface;
    surface = make_shared<PolyMesh>(getVarList());
	bool texturedModel;

	if(surface->ReadOBJModel(objFileName, texturedModel, true))
	{
		// 2. Load a Texture model with parametrization.
		if (texturedModel)
		{
            referenceSurface = make_shared<PolyMesh_AABBTree>(*surface);
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
    BaseMeshCreator<Scalar> baseMeshCreator;

    if(crossMesh->crossList.size() == atBoundary.size())
    {
        for(pCross cross: crossMesh->crossList)
        {
            cross->atBoundary = atBoundary[cross->crossID];
        }
    }
    else{
        baseMeshCreator.recomputeBoundary(crossMesh);
    }

    float   tiltAngle       = getVarList()->template get<float>("tiltAngle");
    AugmentedVectorCreator vectorCreator(getVarList());
    vectorCreator.CreateAugmentedVector(tiltAngle, crossMesh);

    return  true;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::createCrossMesh( bool previewMode, Matrix4 textureMat)
{
    float   tiltAngle       = getVarList()->template get<float>("tiltAngle");
    float   patternID       = getVarList()->template get<int>("patternID");
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
        AugmentedVectorCreator vectorCreator(getVarList());
        if(!vectorCreator.UpdateMeshTiltRange(crossMesh))
        {
            return false;
        }
        return true;
    }

    return false;
}

//**************************************************************************************//
//                                Auxiliary Tree
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