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
#include "tbb/tbb.h"

template <typename Scalar>
CrossMeshCreator<Scalar>::CrossMeshCreator(shared_ptr<InputVarList> var) :TopoObject(var)
{
	referenceSurface = nullptr;

	pattern2D = nullptr;

	crossMesh = nullptr;
    
}
template <typename Scalar>
void CrossMeshCreator<Scalar>::clear()
{
    referenceSurface.reset();
    pattern2D.reset();
    crossMesh.reset();
}

template <typename Scalar>
CrossMeshCreator<Scalar>::CrossMeshCreator(const CrossMeshCreator &_model) : TopoObject(_model)
{
    clear();

    if(_model.pattern2D) pattern2D = make_shared<CrossMesh<Scalar>>(*_model.pattern2D);
    if(_model.referenceSurface) referenceSurface = make_shared<PolyMesh_AABBTree<Scalar>>(*_model.referenceSurface);
    if(_model.crossMesh) crossMesh = make_shared<CrossMesh<Scalar>>(*_model.crossMesh);
}

template <typename Scalar>
CrossMeshCreator<Scalar>::~CrossMeshCreator()
{

}

//**************************************************************************************//
//                         Setup ReferenceSurface/PatternMesh/CrossMesh
//**************************************************************************************//

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
        getVarList()->add(true, "texturedModel", "");
    }
    return referenceSurface != nullptr;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::setPatternMesh(pPolyMesh _pattern)
{
    if(_pattern == nullptr) return false;
    pattern2D = make_shared<CrossMesh<Scalar>>(*_pattern);
    return true;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::setCrossMesh(pCrossMesh _crossmesh)
{
    if(_crossmesh == nullptr) return false;
    crossMesh = make_shared<CrossMesh<Scalar>>(*_crossmesh);
    return  true;
}

//**************************************************************************************//
//                         Update ReferenceSurface/PatternMesh/CrossMesh
//**************************************************************************************//

template <typename Scalar>
bool CrossMeshCreator<Scalar>::createCrossMeshFromRSnPattern( bool previewMode, Matrix4 textureMat)
{
    if(pattern2D && referenceSurface && referenceSurface->texturedModel){
        tbb::tick_count sta = tbb::tick_count::now();
        pPolyMesh baseMesh2D;
        BaseMeshCreator<Scalar> baseMeshCreator(referenceSurface, pattern2D, getVarList());
        baseMeshCreator.computeBaseCrossMesh(textureMat, baseMesh2D, crossMesh, previewMode);
        crossMesh->setBaseMesh2D(baseMesh2D);
        std::cout << "BaseMesh Creation:\t" <<  (tbb::tick_count::now() - sta).seconds() << std::endl;
    }
    return crossMesh != nullptr;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>:: updatePatternMesh()
{
    tbb::tick_count sta = tbb::tick_count::now();
    int patternID = getVarList()->getInt("patternID");
    int patternRadius = getVarList()->getInt("patternRadius");
    PatternCreator<Scalar> patternCreator(getVarList());
    patternCreator.create2DPattern(PatternType(patternID), patternRadius, pattern2D);
    std::cout << "Pattern Creation:\t" <<  (tbb::tick_count::now() - sta).seconds() << std::endl;
    return pattern2D != nullptr;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::createAugmentedVectors(){
    if(crossMesh){
        tbb::tick_count sta = tbb::tick_count::now();
        float tiltAngle = getVarList()->getFloat("tiltAngle");
        AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
        vectorCreator.createAugmentedVectors(tiltAngle, crossMesh);
        std::cout << "Augmented Vectors Creation:\t" <<  (tbb::tick_count::now() - sta).seconds() << std::endl;
        return true;
    }
    return false;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::updateAugmentedVectors(){
    if(crossMesh){
        tbb::tick_count sta = tbb::tick_count::now();
        float tiltAngle = getVarList()->getFloat("tiltAngle");
        AugmentedVectorCreator<Scalar> vectorCreator(getVarList());
        vectorCreator.updateAugmentedVectors(tiltAngle, crossMesh);
        std::cout << "Augmented Vectors Update:\t" <<  (tbb::tick_count::now() - sta).seconds() << std::endl;
        return true;
    }
    return false;
}

template <typename Scalar>
bool CrossMeshCreator<Scalar>::updateCrossMeshBoundary(const vector<int>& boundary_crossIDs){
    if(crossMesh)
    {
        for(size_t id = 0; id < crossMesh->size(); id++){
            crossMesh->cross(id)->atBoundary = false;
        }
        
        for(size_t id = 0; id < boundary_crossIDs.size(); id++)
        {
            int crossID = boundary_crossIDs[id];
            if(crossID >= 0 && crossID < crossMesh->size()){
                crossMesh->cross(crossID)->atBoundary = true;
            }
        }
    }
    return true;
}


template <typename Scalar>
bool CrossMeshCreator<Scalar>::computeAugmentedRange()
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
    if(referenceSurface){
        //the following want to tranform the points in the surface texture space to the pattern space

        // Compute 2D bounding box of the parameterized surface mesh
        Box<Scalar> texBBox = referenceSurface->texBBox();

        //1) centralize the surface texture
        Matrix4 trans1 = Matrix4::Identity();
        trans1(0, 3) = -0.5*(texBBox.minPt.x() + texBBox.maxPt.x());
        trans1(1, 3) = -0.5*(texBBox.minPt.y() + texBBox.maxPt.y());
        trans1(2, 3) = 0;

        //2) scale 1) into [-0.5, -0.5]x [0.5, 0.5]
        Scalar scale_factor = getVarList()->getFloat("textureScaleFactor");
        Scalar footScale = scale_factor / max(texBBox.maxPt.x() - texBBox.minPt.x(), texBBox.maxPt.y() - texBBox.minPt.y());
        Matrix4 scale = Matrix4::Identity();
        scale(0, 0) = footScale; scale(1, 1) = footScale;

        //3) tranform 2) by inveInteractMat
        Matrix4 inveInteractMat = interactMat.inverse();
        //Compatible issue with the old data
        //Reason: Since the scale of 2D pattern space is [-1, 1] while the scale of 2D texture space is [0, 1]
        inveInteractMat(0, 3) /= 2;
        inveInteractMat(1, 3) /= 2;
        inveInteractMat(2, 3) /= 2;

        //4) move the 3)'s center into [0.5, 0.5], so that it is within [0, 0] x[1, 1]
        Matrix4 trans2 = Matrix4::Identity();
        trans2(0, 3) = 0.5;
        trans2(1, 3) = 0.5;
        trans2(2, 3) = 0;

        Matrix4 textureMat = trans2 * inveInteractMat * scale * trans1;
        //5) inverse it because we want a transform form pattern space to surface texture space

        Matrix4 invtextureMat = textureMat.inverse();

        //6)scale the pattern from [-1, 1] x[1, 1] to [0, 1]
        Matrix4 trans3 = Matrix4::Identity();
        trans3(0, 3) = 1;
        trans3(1, 3) = 1;
        trans3(2, 3) = 0;

        Matrix4 scale2 = Matrix4::Identity();
        scale2(0, 0) = 0.5;
        scale2(1, 1) = 0.5;

        return invtextureMat * scale2 * trans3;
    }
    else{
        return Matrix<Scalar, 4, 4>::Identity();
    }

}

template class CrossMeshCreator<double>;
