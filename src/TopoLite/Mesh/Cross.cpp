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
#include "Utility/HelpFunc.h"
#include "Cross.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

Cross::Cross(std::shared_ptr<InputVarList> var) : TopoObject(var)
{
	isVisited = false;
}

Cross::~Cross()
{
	oriPoints.clear();
}

Cross::Cross(const Cross &_cross) : _Polygon(_cross), TopoObject(_cross)
{
    crossID = _cross.crossID;
    atBoundary = _cross.atBoundary;
    for(int id = 0; id < _cross.oriPoints.size(); id++){
        shared_ptr<OrientPoint> oript = make_shared<OrientPoint>(*_cross.oriPoints[id]);
        oriPoints.push_back(oript);
    }
}

void Cross::Print()
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

void Cross::InitTiltNormals()
{
	oriPoints.clear();
	for (int i = 0; i < vers.size(); i++)
	{
		Vector3f staPt = vers[i].pos;
		Vector3f endPt = vers[(i + 1) % vers.size()].pos;
		Vector3f midPt = 0.5f * (staPt + endPt);

		Vector3f edgeDir = (endPt - staPt) / len(endPt - staPt);
		Vector3f midPtDir = midPt - center;

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
		Vector3f tiltNormal = edgeDir CROSS avgNormal;

		float dotP = midPtDir DOT tiltNormal;
		if (dotP < 0 )
		{
			tiltNormal = -1.0f * tiltNormal;
		}

		tiltNormal = tiltNormal / len(tiltNormal);

		// Push back orientPoint
		shared_ptr<OrientPoint> oriPoint =  make_shared<OrientPoint>(midPt, tiltNormal, edgeDir);
		oriPoints.push_back(oriPoint);
	}
}

void Cross::UpdateTiltNormal_Root(float tiltAngle)
{
	for (int i = 0; i < oriPoints.size(); i++)
	{
		Vector3f staPt = vers[i].pos;
		Vector3f endPt = vers[(i + 1) % vers.size()].pos;
		Vector3f edgeDir = (endPt - staPt) / len(endPt - staPt);

		// Upper tilt normal
		if (i % 2 == 0)    oriPoints[i]->tiltSign = TILT_SIGN_POSITIVE;  // Initialize the sign
		else               oriPoints[i]->tiltSign = TILT_SIGN_NEGATIVE;

		float rotAngle = oriPoints[i]->tiltSign * tiltAngle;
		oriPoints[i]->normal = RotateNormal(oriPoints[i]->rotation_base, edgeDir, rotAngle);
		oriPoints[i]->update_rotation(rotAngle);
	}
}

void Cross::UpdateTiltNormal(float tiltAngle)
{
	//////////////////////////////////////////////////////////////////////
	// 1. Update tilt normals for the edges shared with visited neighbors

	int startEdgeID = -1;
	bool ground_touch_bdry = getVarList()->get<bool>("ground_touch_bdry");

	for (int i = 0; i < oriPoints.size(); i++)
	{
		pCross neighbor = neighbors[i].lock();

		if (neighbor == nullptr) continue;

		Vector3f staPt = vers[i].pos;
		Vector3f endPt = vers[(i + 1) % vers.size()].pos;
		Vector3f edgeDir = (endPt - staPt) / len(endPt - staPt);

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
				Vector3f edgeDir = (endPt - staPt) / len(endPt - staPt);

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

Vector3f Cross::RotateNormal(Vector3f normal, Vector3f rotAxis, float rotAngle)
{
    rotAngle = rotAngle / 180 * M_PI;
    Eigen::Matrix3f mat, ux;
    ux <<   0, -rotAxis.z, rotAxis.y,
            rotAxis.z, 0, -rotAxis.x,
            -rotAxis.y, rotAxis.x, 0;
    mat = Eigen::Matrix3f::Identity() * (float)std::cos(rotAngle) + ux * (float)std::sin(rotAngle);
    Eigen::Vector3f inNormal(normal[0], normal[1], normal[2]);
    Eigen::Vector3f outNormal;
    outNormal = mat * inNormal;
    return Vector3f(outNormal[0], outNormal[1], outNormal[2]);
}


//**************************************************************************************//
//                                 Compute Neighbor
//**************************************************************************************//

int Cross::GetNeighborEdgeID(int currCrossID)
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

int Cross::GetVertexEdgeID(int vertexID) {
    for (int i = 0; i < verIDs.size(); i++)
    {
        if(verIDs[i] == vertexID){
            return i;
        }
    }
    return NONE_ELEMENT;
}

int Cross::GetPrevEdgeID(int edgeID){
    return (edgeID - 1 + verIDs.size()) % verIDs.size();
}

int Cross::GetSharedCross(weak_ptr<Cross> ncross)
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

int Cross::GetShareEdge(weak_ptr<Cross> ncross) {
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