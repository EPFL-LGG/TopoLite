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

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef shared_ptr<VTex<Scalar>> pVTex;

private:

	vector<pCross> crossList;                   // Cross list of the mesh

	vector<vector<wpCross>> vertexCrossList; 	// all cross around a vertex

public:

	shared_ptr<PolyMesh<Scalar>> baseMesh2D;    // the 2D tiling pattern

public:

    CrossMesh(std::shared_ptr<InputVarList> var);

    CrossMesh(const CrossMesh &_cross);

    CrossMesh(const PolyMesh<Scalar> &polyMesh);

    ~CrossMesh();

public:

    void setPolyMesh(const PolyMesh<Scalar> &polyMesh);

    void setBaseMesh2D(shared_ptr<PolyMesh<Scalar>> _baseMesh2D);

    void createConnectivity();

    void print() const;

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
        if(index >= 0 || index < crossList.size())
        {
            crossList[index] = _cross;
            PolyMesh<Scalar>::polyList[index] = _cross;
        }
    }

    pCross cross(size_t index)
    {
        if(index >= 0 || index < crossList.size()){
            return crossList[index];
        }
        return nullptr;
    }

    size_t size() const{
        return crossList.size();
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

#include "CrossMesh.cpp"

#endif

