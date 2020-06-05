///////////////////////////////////////////////////////////////
//
// Cross.h
//
//   Cross section
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 15/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _CROSS_MESH_H
#define _CROSS_MESH_H

#include "TopoLite/Mesh/PolyMesh.h"
#include "Cross.h"

template<typename Scalar>
class CrossMesh : private PolyMesh<Scalar>
{

public:

    using pCross = shared_ptr<Cross<Scalar>>;

    using wpCross = weak_ptr<Cross<Scalar>>;

    using pPolygon = shared_ptr<_Polygon<Scalar>> ;

    using pTriangle = shared_ptr<Triangle<Scalar>> ;

    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

    typedef Eigen::Matrix<Scalar, 2, 1> Vector2;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef shared_ptr<VTex<Scalar>> pVTex;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

private:

	vector<pCross> crossList;                   // Cross list of the mesh

	vector<vector<wpCross>> vertexCrossList; 	// all cross around a vertex

public:

    pPolyMesh baseMesh2D;    // the 2D tiling pattern

public:

    CrossMesh(std::shared_ptr<InputVarList> var);

    CrossMesh(const CrossMesh &_cross);

    CrossMesh(const PolyMesh<Scalar> &polyMesh);

    ~CrossMesh();

public:

    void setPolyMesh(const PolyMesh<Scalar> &polyMesh);

    void update();

    void setBaseMesh2D(shared_ptr<PolyMesh<Scalar>> _baseMesh2D);

    void createConnectivity();

    void print() const;

    nlohmann::json dump() const;

    void clear()
    {
        PolyMesh<Scalar>::clear();
        crossList.clear();
        vertexCrossList.clear();
        baseMesh2D.reset();
    }

    void push_back(pCross _cross)
    {
        crossList.push_back(_cross);
        PolyMesh<Scalar>::polyList.push_back(_cross);
    }

    void set_cross(size_t index, pCross _cross)
    {
        if(index >= 0 && index < crossList.size())
        {
            crossList[index] = _cross;
            PolyMesh<Scalar>::polyList[index] = _cross;
        }
    }

    pCross cross(size_t index)
    {
        if(index >= 0 && index < crossList.size()){
            return crossList[index];
        }
        return nullptr;
    }

    void erase(size_t index){
        if(index >= 0 && index < crossList.size()){
            //erase crossList
            crossList[index].reset();
            crossList[index] = nullptr;
            //erase polyList
            PolyMesh<Scalar>::polyList[index].reset();
            PolyMesh<Scalar>::polyList[index] = nullptr;
        }
    }

    void erase_nullptr();

    size_t size() const{
        return crossList.size();
    }

    const vector<wpCross>& getVertexCrosses(int verID){
        if(verID >= 0 && verID < vertexCrossList.size()){
            return vertexCrossList[verID];
        }
        static const vector<wpCross> none;
        return none;
    }

    pPolyMesh getPolyMesh() const{
        pPolyMesh polymesh = make_shared<PolyMesh<Scalar>>(*(PolyMesh<Scalar> *) this);
        return polymesh;
    }

    vector<int> getBoundaryCrossIDs() const{
        vector<int> boundary_crossIDs;
        for(size_t id = 0; id < crossList.size(); id++){
            if(crossList[id]->atBoundary){
                boundary_crossIDs.push_back(id);
            }
        }
        return boundary_crossIDs;
    }

public:

    using PolyMesh<Scalar>::getVertices;
    using PolyMesh<Scalar>::getVarList;
    using PolyMesh<Scalar>::setVarList;

    using PolyMesh<Scalar>::volume;
    using PolyMesh<Scalar>::centroid;
    using PolyMesh<Scalar>::bbox;
    using PolyMesh<Scalar>::texBBox;
    using PolyMesh<Scalar>::lowestPt;

    using PolyMesh<Scalar>::vertexList;
    using PolyMesh<Scalar>::readOBJModel;
    using PolyMesh<Scalar>::writeOBJModel;
    using PolyMesh<Scalar>::update;

public:

	void updateCrossID();

    Scalar computeAverageCrossSize() const;

private:

    pCross getNeighbor(pCross cross, pVertex v0, pVertex v1) const;


};
#endif

