///////////////////////////////////////////////////////////////
//
// CrossMesh.cpp
//
//   Cross Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 15/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#include "Utility/HelpDefine.h"
#include "CrossMesh.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//


template<typename Scalar>
CrossMesh<Scalar>::CrossMesh(std::shared_ptr<InputVarList> var)
:PolyMesh<Scalar>::PolyMesh(var)
{
	clear();
}

template<typename Scalar>
CrossMesh<Scalar>::~CrossMesh()
{
    clear();
}

template<typename Scalar>
CrossMesh<Scalar>::CrossMesh(const CrossMesh<Scalar> &crossMesh)
:PolyMesh<Scalar>(crossMesh.getVarList())
{
    clear();

    // create cross geometry
    for(size_t id = 0; id < crossMesh.crossList.size(); id++)
    {
        pCross cross = make_shared<Cross<Scalar>>(*crossMesh.crossList[id]);
        crossList.push_back(cross);
        PolyMesh<Scalar>::polyList.push_back(cross);
    }

    PolyMesh<Scalar>::computeVertexList();
    PolyMesh<Scalar>::computeTextureList();

    createConnectivity();
}

template<typename Scalar>
CrossMesh<Scalar>::CrossMesh(const PolyMesh<Scalar> &polyMesh)
:PolyMesh<Scalar>::PolyMesh(polyMesh.getVarList())
{
    clear();

    setPolyMesh(polyMesh);
}

template<typename Scalar>
void CrossMesh<Scalar>::setPolyMesh(const PolyMesh<Scalar> &polyMesh)
{
    clear();

    setVarList(polyMesh.getVarList());

    // 1) create Cross
    for(size_t id = 0; id < polyMesh.polyList.size(); id++)
    {
        pCross cross = make_shared<Cross<Scalar>>(*polyMesh.polyList[id], polyMesh.getVarList());
        cross->crossID = id;
        crossList.push_back(cross);
        PolyMesh<Scalar>::polyList.push_back(cross);
    }

    PolyMesh<Scalar>::computeVertexList();
    PolyMesh<Scalar>::computeTextureList();
    PolyMesh<Scalar>::removeDuplicatedVertices();

    createConnectivity();
}

template<typename Scalar>
void CrossMesh<Scalar>::update(){
    PolyMesh<Scalar>::computeTextureList();
    PolyMesh<Scalar>::removeDuplicatedVertices();
    createConnectivity();
}

template<typename Scalar>
void CrossMesh<Scalar>::createConnectivity()
{
    // 0) initialize
    updateCrossID();
    vertexCrossList.clear();

    // 1) create vertexCrossList
    vertexCrossList.resize(PolyMesh<Scalar>::vertexList.size());
    for(pCross cross: crossList){
        for(pVertex vertex: cross->vers){
            vertexCrossList[vertex->verID].push_back(cross);
        }
    }

    // 2) create neighbors
    for(pCross cross: crossList)
    {
        cross->neighbors.clear();
        for(size_t id = 0; id < cross->vers.size(); id++)
        {
            pVertex curr_ver = cross->vers[id];
            pVertex next_ver = cross->vers[(id + 1) % cross->vers.size()];
            pCross neighbor = getNeighbor(cross, curr_ver, next_ver);
            if(neighbor != nullptr){
                cross->neighbors.push_back(neighbor);
            }
            else{
                cross->neighbors.push_back(shared_ptr<Cross<Scalar>>());
            }
        }
    }
}

template<typename Scalar>
shared_ptr<Cross<Scalar>> CrossMesh<Scalar>::getNeighbor(pCross cross,
                                                         CrossMesh::pVertex v0,
                                                         CrossMesh::pVertex v1) const
{

    if(v0 == nullptr || v1 == nullptr || cross == nullptr)
        return nullptr;

    vector<int> crossIDs;
    for(wpCross neighbor: vertexCrossList[v0->verID])
    {
        if(neighbor.lock()->crossID != cross->crossID)
            crossIDs.push_back(neighbor.lock()->crossID);
    }

    for(wpCross neighbor: vertexCrossList[v1->verID])
    {
        if(neighbor.lock()->crossID != cross->crossID)
            crossIDs.push_back(neighbor.lock()->crossID);
    }

    std::sort(crossIDs.begin(), crossIDs.end());


    for(int id = 0; id < (int)(crossIDs.size()) - 1; id ++){
        if(crossIDs[id] == crossIDs[id + 1])
        {
            return crossList[crossIDs[id]];
        }
    }
    return nullptr;
}

template<typename Scalar>
void CrossMesh<Scalar>::print() const
{
	printf("cross num: %lu \n", crossList.size());

	for (size_t i = 0; i < crossList.size(); i++)
	{
        crossList[i]->print();
	}
}

template<typename Scalar>
void CrossMesh<Scalar>::setBaseMesh2D(shared_ptr<PolyMesh<Scalar>> _baseMesh2D)
{
	baseMesh2D = _baseMesh2D;
}


//**************************************************************************************//
//                                   Update Vertices
//**************************************************************************************//


template<typename Scalar>
Scalar CrossMesh<Scalar>::computeAverageCrossSize() const
{
    Scalar area = 0;
    for(pCross cross: crossList){
        area += cross->area();
    }
    if(crossList.empty()){
        return 0;
    }
    return area / crossList.size();
}

template<typename Scalar>
void CrossMesh<Scalar>::updateCrossID()
{
    for(size_t id = 0; id < crossList.size(); id++){
        if(crossList[id]) crossList[id]->crossID = id;
    }
}

template<typename Scalar>
void CrossMesh<Scalar>::erase_nullptr()
{
    //remove crossList
    for(auto it = crossList.begin(); it != crossList.end();){
        if(*it == nullptr){
            it = crossList.erase(it);
        }
        else{
            it ++;
        }
    }

    //remove polyList
    for(auto it = PolyMesh<Scalar>::polyList.begin(); it != PolyMesh<Scalar>::polyList.end();){
        if(*it == nullptr){
            it = PolyMesh<Scalar>::polyList.erase(it);
        }
        else{
            it ++;
        }
    }

    //remove vertexList and vertexCrossList
    typename vector<vector<wpCross>>::iterator vcross_it = vertexCrossList.begin();
    typename vector<pVertex>::iterator vertex_it = vertexList.begin();
    for(;vcross_it != vertexCrossList.end(); ){
        for(auto it = vcross_it->begin(); it != vcross_it->end(); ){
            if(it->lock() == nullptr){
                vcross_it->erase(it);
            }
            else{
                it ++;
            }
        }

        if(vcross_it->empty()){
            vcross_it = vertexCrossList.erase(vcross_it);
            vertex_it = vertexList.erase(vertex_it);
        }
        else{
            vcross_it ++;
            vertex_it ++;
        }
    }

    for(int id = 0; id < vertexList.size(); id++){
        vertexList[id]->verID = id;
    }

    updateCrossID();
}


template class CrossMesh<double>;
template class CrossMesh<float>;



