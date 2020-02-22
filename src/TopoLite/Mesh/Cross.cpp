///////////////////////////////////////////////////////////////
//
// Cross.cpp
//
//   Cross polygon with two sets of tilt normals for constructing a non-convex polyhedron
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
///////////////////////////////////////////////////////////////

#include "Utility/math3D.h"
#include "Utility/HelpDefine.h"
#include "Cross.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

template <typename Scalar>
Cross<Scalar>::Cross(std::shared_ptr<InputVarList> var) : TopoObject(var)
{
	isVisited = false;
	crossID = -1;
}

template <typename Scalar>
Cross<Scalar>::~Cross()
{
	oriPoints.clear();
	crossID = -1;
}

template <typename Scalar>
Cross<Scalar>::Cross(const Cross &_cross) : _Polygon<Scalar>(_cross), TopoObject(_cross)
{
    //copy data
    crossID = _cross.crossID;
    atBoundary = _cross.atBoundary;
    isVisited = _cross.isVisited;

    for(int id = 0; id < _cross.oriPoints.size(); id++)
    {
        shared_ptr<OrientPoint<Scalar>> oript = make_shared<OrientPoint<Scalar>>(*_cross.oriPoints[id]);
        oriPoints.push_back(oript);
    }

    //copy neighbors
    //theorically we should not copy the neighbor,
    //because it is not recommanded to copy pointer
    //but for practical reason, we should copy it.
    //just don't forgot to update the neighbor pointer list
    //once the old neighbor pointers are not more valid

    neighbors = _cross.neighbors;
}

template <typename Scalar>
void Cross<Scalar>::print()
{
	printf("oriPoints num: %lu \n", oriPoints.size());
	for (int i = 0; i < oriPoints.size(); i++)
	{
        oriPoints[i]->print();
	}
	printf("\n");
}

//**************************************************************************************//
//                                 Compute Tilt Normals
//**************************************************************************************//

template <typename Scalar>
void Cross<Scalar>::initTiltNormals()
{

    //1) recompute normal and center for safety reason
    oriPoints.clear();
    _Polygon<Scalar>::computeNormal();
    _Polygon<Scalar>::computeCenter();

    //2)
	const vector<pVertex> &vers= _Polygon<Scalar>::vers;
	const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;

	int verN = vers.size();
	for (int i = 0; i < verN; i++)
	{
		Vector3 staPt = vers[i]->pos;
		Vector3 endPt = vers[(i + 1) % verN]->pos;
		Vector3 midPt = (staPt + endPt) / 2;

        Vector3 edgeDir = (endPt - staPt).normalized();
        Vector3 midPtDir = midPt - center;

		// Average edge normal
		Vector3 avgNormal;
		if( neighbors.size() > 0 )
		{
			if ( neighbors[i].lock() != nullptr)    avgNormal = (normal + neighbors[i].lock()->normal) / 2.0f;
			else                         	avgNormal = normal;
		}
		else
		{
			avgNormal = normal;   // For the case that the input mesh is a single polygon
		}

		// Initial tilt normal (perpendicular to edge and edge normal; pointing outward)
		Vector3 tiltNormal = edgeDir.cross(avgNormal).normalized();

		if (midPtDir.dot(tiltNormal) < 0 )
		{
			tiltNormal = -tiltNormal;
		}

		// Push back orientPoint
		shared_ptr<OrientPoint<Scalar>> oriPoint =  make_shared<OrientPoint<Scalar>>(midPt, tiltNormal, edgeDir);
		oriPoints.push_back(oriPoint);
	}
}

template <typename Scalar>
void Cross<Scalar>::updateTiltNormalsRoot(float tiltAngle)
{
    const vector<pVertex> &vers= _Polygon<Scalar>::vers;
    const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;

	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3 staPt = vers[i]->pos;
		Vector3 endPt = vers[(i + 1) % vers.size()]->pos;
		Vector3 edgeDir = (endPt - staPt).normalized();

		// Upper tilt normal
		if (i % 2 == 0)    oriPoints[i]->tiltSign = TILT_SIGN_POSITIVE;  // Initialize the sign
		else               oriPoints[i]->tiltSign = TILT_SIGN_NEGATIVE;

		float rotAngle = oriPoints[i]->tiltSign * tiltAngle;
		oriPoints[i]->updateAngle(rotAngle);
	}
}

template <typename Scalar>
void Cross<Scalar>::updateTiltNormals(float tiltAngle)
{
    const vector<pVertex> &vers= _Polygon<Scalar>::vers;
    const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;
	bool boundary_not_tilt = getVarList()->template get<bool>("ground_touch_bdry");

	//1) Clear all signs
	for(int id = 0; id < oriPoints.size(); id++){
	    ori(id)->tiltSign = TILT_SIGN_NONE;
	}

    int startEdgeID = -1;
    int num_edges_assigned = 0;
	//2) If neighbor has a sign, reverse it
	for (int i = 0; i < oriPoints.size(); i++)
	{

	    //1) check neighbor exist and has been visited
	    if(neighbors.size() <= i) break;
		pCross neighbor = neighbors[i].lock();
		if (neighbor == nullptr) continue;
		if(neighbor->isVisited == false) continue;

        //2)reverse the tilt sign of its neighbor
        int neiborEdgeID = neighbor->getEdgeIDOfGivenCross(this);
        TopoASSERT(neiborEdgeID != NONE_ELEMENT);
        if(neiborEdgeID == NONE_ELEMENT) continue;

        //3)reverse the tilt sign of its neighbor
        oriPoints[i]->tiltSign = (-1) * neighbor->oriPoints[neiborEdgeID]->tiltSign; // Reverse the sign
        if(atBoundary && neighbor->atBoundary && boundary_not_tilt)
        {
            oriPoints[i]->updateAngle(0);
        }
        else{
            oriPoints[i]->updateAngle(tiltAngle);
        }
        num_edges_assigned ++;
        startEdgeID = i;
	}


	//3) Update tilt normals for the remaining edges
	int currEdgeID = startEdgeID;
	while (num_edges_assigned < oriPoints.size())
	{
		// Process edges that have not been updated
		int nextEdgeID = (currEdgeID + 1) % oriPoints.size();

		if (oriPoints[nextEdgeID]->tiltSign == TILT_SIGN_NONE)
		{
            // Reverse the sign
		    if(currEdgeID != -1){
                oriPoints[nextEdgeID]->tiltSign = -1 * oriPoints[currEdgeID]->tiltSign;
            }
			else{
			    //force it to be positive
			    oriPoints[nextEdgeID]->tiltSign = TILT_SIGN_POSITIVE;
            }

            if(atBoundary && checkNeighborAtBoundary(nextEdgeID) && boundary_not_tilt){
                oriPoints[nextEdgeID]->updateAngle(0);
            }
            else{
                oriPoints[nextEdgeID]->updateAngle(tiltAngle);
            }

            num_edges_assigned++;
		}
		currEdgeID = nextEdgeID;
	}
}

//**************************************************************************************//
//                                 Compute Neighbor
//**************************************************************************************//

template <typename Scalar>
int Cross<Scalar>::getEdgeIDOfGivenCross(const Cross<Scalar>* ncross)
{
    if(ncross == nullptr) return NONE_ELEMENT;

	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors[i].lock() == nullptr)
			continue;

		if (neighbors[i].lock()->crossID == ncross->crossID)
			return i;
	}

	return NONE_ELEMENT;
}

template <typename Scalar>
int Cross<Scalar>::getEdgeIDOfGivenVertexID(int vertexID)
{
    const vector<pVertex> &vers= _Polygon<Scalar>::vers;
    for (int i = 0; i < vers.size(); i++)
    {
        if(vers[i]->verID == vertexID){
            return i;
        }
    }
    return NONE_ELEMENT;
}

template <typename Scalar>
int Cross<Scalar>::getPrevEdgeID(int edgeID){
    const vector<pVertex> &vers= _Polygon<Scalar>::vers;
    return (edgeID - 1 + vers.size()) % vers.size();
}

template <typename Scalar>
int Cross<Scalar>::getCrossIDsSharedWithCross(const Cross<Scalar>* ncross, vector<int> &shared_crossIDs)
{
    if(ncross == nullptr) return NONE_ELEMENT;

	vector<int> neigbor_crossID;
	for(int id = 0; id < neighbors.size(); id++)
	{
		if(neighbors[id].lock()) neigbor_crossID.push_back(neighbors[id].lock()->crossID);
	}

	for(int id = 0; id < ncross->neighbors.size(); id++)
	{
		if(ncross->neighbors[id].lock()) neigbor_crossID.push_back(ncross->neighbors[id].lock()->crossID);
	}

	std::sort(neigbor_crossID.begin(), neigbor_crossID.end());


    shared_crossIDs.clear();
	for(int id = 0; id < (int)(neigbor_crossID.size()) - 1; id ++){
		if(neigbor_crossID[id] == neigbor_crossID[id + 1])
		{
            shared_crossIDs.push_back(neigbor_crossID[id]);
		}
	}

	if(shared_crossIDs.empty()){
        return NONE_ELEMENT;
    }
	else{
	    return shared_crossIDs.size();
	}

}

template <typename Scalar>
int Cross<Scalar>::getEdgeIDSharedWithCross(const Cross<Scalar>* ncross)
{
    if(ncross == nullptr) return NONE_ELEMENT;

    const vector<pVertex> &vers= _Polygon<Scalar>::vers;

    map<int, bool> neighborVertex;
    for(int id = 0; id < ncross->vers.size(); id++){
        neighborVertex[ncross->vers[id]->verID] = true;
    }

    for(int id = 0; id < vers.size(); id++){
        int vID = vers[id]->verID;
        int nvID = vers[(id + 1) % vers.size()]->verID;
        if(neighborVertex[vID] && neighborVertex[nvID]){
            return id;
        }
    }
    return NONE_ELEMENT;
}

template<typename Scalar>
bool Cross<Scalar>::checkNeighborAtBoundary(int nID)
{
    //neighbor is not existed
    if(nID >= neighbors.size() || nID < 0) return true;
    if(neighbors.at(nID).lock() == nullptr) return true;

    //neighbor is existed
    return neighbors.at(nID).lock()->atBoundary;
}
