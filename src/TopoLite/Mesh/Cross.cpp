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
}

template <typename Scalar>
Cross<Scalar>::~Cross()
{
	oriPoints.clear();
}

template <typename Scalar>
Cross<Scalar>::Cross(const Cross &_cross) : _Polygon<Scalar>(_cross), TopoObject(_cross)
{
    crossID = _cross.crossID;
    atBoundary = _cross.atBoundary;
    for(int id = 0; id < _cross.oriPoints.size(); id++)
    {
        shared_ptr<OrientPoint<Scalar>> oript = make_shared<OrientPoint>(*_cross.oriPoints[id]);
        oriPoints.push_back(oript);
    }
}

template <typename Scalar>
void Cross<Scalar>::Print()
{
	printf("oriPoints num: %lu \n", oriPoints.size());
	for (int i = 0; i < oriPoints.size(); i++)
	{
        oriPoints[i]->Print();
	}
	printf("\n");
}

//**************************************************************************************//
//                                 Compute Tilt Normals
//**************************************************************************************//

template <typename Scalar>
void Cross<Scalar>::InitTiltNormals()
{
	oriPoints.clear();

	const vector<_Vertex<Scalar>> &vers= _Polygon<Scalar>::vers;
	const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;

	int verN = vers.size();
	for (int i = 0; i < verN; i++)
	{
		Vector3 staPt = vers[i].pos;
		Vector3 endPt = vers[(i + 1) % verN].pos;
		Vector3 midPt = (staPt + endPt) / 2;

        Vector3 edgeDir = (endPt - staPt).normalized();
        Vector3 midPtDir = midPt - center;

		// Average edge normal
		Vector3f avgNormal;
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
		Vector3f tiltNormal = edgeDir.cross(avgNormal);

		float dotP = midPtDir.dot(tiltNormal);
		if (dotP < 0 )
		{
			tiltNormal = -tiltNormal;
		}

		tiltNormal.normalize();

		// Push back orientPoint
		shared_ptr<OrientPoint<Scalar>> oriPoint =  make_shared<OrientPoint<Scalar>>(midPt, tiltNormal, edgeDir);
		oriPoints.push_back(oriPoint);
	}
}

template <typename Scalar>
void Cross<Scalar>::UpdateTiltNormal_Root(float tiltAngle)
{
    const vector<_Vertex<Scalar>> &vers= _Polygon<Scalar>::vers;
    const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;

	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3f staPt = vers[i].pos;
		Vector3f endPt = vers[(i + 1) % vers.size()].pos;
		Vector3f edgeDir = (endPt - staPt).normalized();

		// Upper tilt normal
		if (i % 2 == 0)    oriPoints[i]->tiltSign = TILT_SIGN_POSITIVE;  // Initialize the sign
		else               oriPoints[i]->tiltSign = TILT_SIGN_NEGATIVE;

		float rotAngle = oriPoints[i]->tiltSign * tiltAngle;
		oriPoints[i]->normal = RotateNormal(oriPoints[i]->rotation_base, edgeDir, rotAngle);
		oriPoints[i]->update_rotation(rotAngle);
	}
}

template <typename Scalar>
void Cross<Scalar>::UpdateTiltNormal(float tiltAngle)
{
	//////////////////////////////////////////////////////////////////////
	// 1. Update tilt normals for the edges shared with visited neighbors

    const vector<_Vertex<Scalar>> &vers= _Polygon<Scalar>::vers;
    const Vector3 &center = _Polygon<Scalar>::center;
    const Vector3 &normal = _Polygon<Scalar>::normal;

	int startEdgeID = -1;
	bool ground_touch_bdry = getVarList()->template get<bool>("ground_touch_bdry");

	for (int i = 0; i < oriPoints.size(); i++)
	{
		pCross neighbor = neighbors[i].lock();

		if (neighbor == nullptr) continue;

		Vector3f staPt = vers[i].pos;
		Vector3f endPt = vers[(i + 1) % vers.size()].pos;
		Vector3f edgeDir = (endPt - staPt).normalized();

		if(neighbor->isVisited == true)
		{
			int neiborEdgeID = neighbor->GetNeighborEdgeID(crossID);
			if (neiborEdgeID == NONE_ELEMENT)
			{
				printf("Warning: neiborEdgeID should be -1. \n");
				continue;
			}

			//tilt normal
			oriPoints[i]->tiltSign = -1 * neighbor->oriPoints[neiborEdgeID]->tiltSign; // Reverse the sign
			if(atBoundary && neighbor->atBoundary && ground_touch_bdry)
			{
				float rotAngle = 0;
				oriPoints[i]->normal =
				        RotateNormal(oriPoints[i]->rotation_base, edgeDir, rotAngle);
				oriPoints[i]->update_rotation(rotAngle);
			}
			else{
				float rotAngle = oriPoints[i]->tiltSign * tiltAngle;
				oriPoints[i]->normal = RotateNormal(oriPoints[i]->rotation_base, edgeDir, rotAngle);
				oriPoints[i]->update_rotation(rotAngle);
			}

			startEdgeID = i;
		}
	}

	//////////////////////////////////////////////////////////////////////
	// 2. Update tilt normals for the remaining edges

	int currEdgeID = startEdgeID;
	while (true)
	{
		// Check if all edges have been updated
		bool isAllAsigned = true;
		for (int i = 0; i < oriPoints.size(); i++)
		{
			if (oriPoints[i]->tiltSign == TILT_SIGN_NONE)
			{
				isAllAsigned = false;
				break;
			}
		}

		if (isAllAsigned)
			break;

		// Process edges that have not been updated
		int nextEdgeID = (currEdgeID + 1) % oriPoints.size();

		bool neighbor_at_boundary = false;
		if(neighbors[nextEdgeID].lock() == nullptr)  neighbor_at_boundary = true;
		else if(neighbors[nextEdgeID].lock()->atBoundary) neighbor_at_boundary = true;

		if (oriPoints[nextEdgeID]->tiltSign == TILT_SIGN_NONE)
		{
				Vector3f staPt = vers[nextEdgeID].pos;
				Vector3f endPt = vers[(nextEdgeID + 1) % vers.size()].pos;
				Vector3f edgeDir = (endPt - staPt).normalized();

				// Upper tilt normal
				if(currEdgeID != -1)
					oriPoints[nextEdgeID]->tiltSign = -1 * oriPoints[currEdgeID]->tiltSign; // Reverse the sign
				else
					oriPoints[nextEdgeID]->tiltSign = TILT_SIGN_POSITIVE;

				if (!neighbor_at_boundary || !atBoundary || !ground_touch_bdry)
				{
					float rotAngle = oriPoints[nextEdgeID]->tiltSign * tiltAngle;
					oriPoints[nextEdgeID]->normal = RotateNormal(oriPoints[nextEdgeID]->rotation_base, edgeDir, rotAngle);
					oriPoints[nextEdgeID]->update_rotation(rotAngle);
				}
				else{
					float rotAngle = 0;
					oriPoints[nextEdgeID]->normal = RotateNormal(oriPoints[nextEdgeID]->rotation_base, edgeDir, rotAngle);
					oriPoints[nextEdgeID]->update_rotation(rotAngle);
				}
		}

		currEdgeID = nextEdgeID;
	}
}

template <typename Scalar>
Matrix<Scalar, 3 ,1> Cross<Scalar>::RotateNormal(Vector3 normal, Vector3 rotAxis, Scalar rotAngle)
{
    rotAngle = rotAngle / 180 * M_PI;
    Eigen::Matrix<Scalar, 3, 3> mat, ux;
    ux <<   0, -rotAxis.z(), rotAxis.y(),
            rotAxis.z(), 0, -rotAxis.x(),
            -rotAxis.y(), rotAxis.x(), 0;
    mat = Eigen::Matrix3f::Identity() * std::cos(rotAngle) + ux * std::sin(rotAngle);
    return mat * normal;
}


//**************************************************************************************//
//                                 Compute Neighbor
//**************************************************************************************//

template <typename Scalar>
int Cross<Scalar>::GetNeighborEdgeID(int currCrossID)
{
	for (int i = 0; i < neighbors.size(); i++)
	{
		if (neighbors[i].lock() == nullptr)
			continue;

		if (neighbors[i].lock()->crossID == currCrossID)
			return i;
	}

	return NONE_ELEMENT;
}

template <typename Scalar>
int Cross<Scalar>::GetVertexEdgeID(int vertexID)
{
    const vector<_Vertex<Scalar>> &verIDs= _Polygon<Scalar>::verIDs;
    for (int i = 0; i < verIDs.size(); i++)
    {
        if(verIDs[i] == vertexID){
            return i;
        }
    }
    return NONE_ELEMENT;
}

template <typename Scalar>
int Cross<Scalar>::GetPrevEdgeID(int edgeID){
    const vector<_Vertex<Scalar>> &verIDs= _Polygon<Scalar>::verIDs;
    return (edgeID - 1 + verIDs.size()) % verIDs.size();
}

template <typename Scalar>
int Cross<Scalar>::GetSharedCross(weak_ptr<Cross> ncross)
{
	vector<int> neigbor_crossID;
	for(int id = 0; id < neighbors.size(); id++){
		if(neighbors[id].lock()) neigbor_crossID.push_back(neighbors[id].lock()->crossID);
	}

	for(int id = 0; id < ncross.lock()->neighbors.size(); id++){
		if(ncross.lock()->neighbors[id].lock()) neigbor_crossID.push_back(ncross.lock()->neighbors[id].lock()->crossID);
	}

	std::sort(neigbor_crossID.begin(), neigbor_crossID.end());

	for(int id = 0; id < (int)(neigbor_crossID.size()) - 1; id ++){
		if(neigbor_crossID[id] == neigbor_crossID[id + 1])
			return neigbor_crossID[id];
	}

	return -1;
}

template <typename Scalar>
int Cross<Scalar>::GetShareEdge(weak_ptr<Cross> ncross)
{
    const vector<_Vertex<Scalar>> &verIDs= _Polygon<Scalar>::verIDs;

    map<int, bool> neighborVertex;
    for(int id = 0; id < ncross.lock()->verIDs.size(); id++){
        neighborVertex[ncross.lock()->verIDs[id]] = true;
    }

    for(int id = 0; id < verIDs.size(); id++){
        int vID = verIDs[id];
        int nvID = verIDs[(id + 1) % verIDs.size()];
        if(neighborVertex[vID] && neighborVertex[nvID]){
            return id;
        }
    }
    return -1;
}