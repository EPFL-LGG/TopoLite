///////////////////////////////////////////////////////////////
//
// Remesh_Own.cpp
//
//  Convert Polygonal Mesh into Cross Mesh
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 02/Feb/2018
//
//
///////////////////////////////////////////////////////////////

#include "time.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/HEdgeMesh.h"
#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "AugmentedVectorCreator.h"
#include "CrossConstraint.h"
#include "IO/InputVar.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

AugmentedVectorCreator::~AugmentedVectorCreator()
{

}


//**************************************************************************************//
//                           Convert Polygonal Mesh into Cross Mesh
//**************************************************************************************//

void AugmentedVectorCreator::CreateAugmentedVector(float tiltAngle, pCrossMesh &crossMesh)
{
	if(crossMesh)
	{
	    InitMeshTiltNormals(crossMesh);
        InitMeshTiltNormalsResolveConflicts(crossMesh, tiltAngle);
    }
}


void AugmentedVectorCreator::InitMeshTiltNormals(pCrossMesh crossMesh)
{
	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		shared_ptr<Cross> cross = crossMesh->crossList[i];

		cross->InitTiltNormals();
	}
}


void AugmentedVectorCreator::InitMeshTiltNormalsResolveConflicts(pCrossMesh crossMesh, float tiltAngle)
{
	vector<pCross> bfsQueue;

	if(crossMesh->crossList.empty())
		return;
	// Find a root for Breadth-First Search Traversal

	shared_ptr<Cross> root;
	for(int id = 0; id < crossMesh->crossList.size(); id++){
		if(!crossMesh->crossList[id]->atBoundary)
        {
            root = crossMesh->crossList[id];
            break;
        }

	}

	if(root == nullptr)
		return;

	root->UpdateTiltNormal_Root(tiltAngle);
	root->isVisited = true;
	bfsQueue.push_back(root);

	//for (int i = 0; i < root->neighbors.size(); i++)
	//{
	//	printf(" i=%d \n", i);
	//	root->neighbors[i]->CorrectTiltNormal(tiltAngleUpper);
	//}

	//return;

	// Start Breadth-First Search Traversal
	while (true)
		// Bug: Segmentation fault here; need to check the size of bfsQueue is non zero; recommand to change the traversal method to iterating all tessellation cells
	{

        if(bfsQueue.empty()) break;

		shared_ptr<Cross> currVisit = bfsQueue[0];
		bfsQueue.erase(bfsQueue.begin());

		// Quit if all crosses have been visited
		bool isAllVisited = true;
		for (int i = 0; i < crossMesh->crossList.size(); i++)
		{
			pCross cross = crossMesh->crossList[i];
			if(cross == nullptr)
                continue;
			if (cross->isVisited == false)
			{
				isAllVisited = false;
				break;
			}
		}

		if (isAllVisited)
		{
			break;
		}

		// Process the neighbors of current cross
		for (int i = 0; i < currVisit->neighbors.size(); i++)
		{
			//printf(" i=%d \n", i);
			shared_ptr<Cross> nextVisit = currVisit->neighbors[i].lock();

			if (nextVisit == NULL)
				continue;

//			if(nextVisit->crossID == 3){
//                printf("stop");
//			}

			if(nextVisit->atBoundary && getVarList()->get<bool>("ground_touch_bdry"))
				continue;

			if (nextVisit->isVisited == true)
				continue;

			nextVisit->UpdateTiltNormal(tiltAngle);

			nextVisit->isVisited = true;

			bfsQueue.push_back(nextVisit);
		}
	}

	if(getVarList()->get<bool>("ground_touch_bdry")){
        for(int id = 0; id < crossMesh->crossList.size(); id++)
        {
            if(crossMesh->crossList[id]->atBoundary)
                crossMesh->crossList[id]->UpdateTiltNormal(tiltAngle);
        }
	}
}

void AugmentedVectorCreator::UpdateMeshTiltNormals(pCrossMesh crossMesh, float tiltAngle)
{

    if(crossMesh == nullptr)
        return;

    int m = crossMesh->crossList.size();
    for(int id = 0; id < m; id++)
    {
        shared_ptr<Cross> cross = crossMesh->crossList[id];
        if(cross == nullptr) continue;
        for(int jd = 0; jd < cross->oriPoints.size(); jd++)
        {
            shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
            if(cross->neighbors[jd].lock() != nullptr){
                if(cross->neighbors[jd].lock()->atBoundary == false || cross->atBoundary == false){
                    oriPt->normal = cross->RotateNormal(oriPt->rotation_base, oriPt->rotation_axis, tiltAngle * oriPt->tiltSign);
                    oriPt->update_rotation(tiltAngle);
                }
            }
        }
    }

    return;
}


bool AugmentedVectorCreator::UpdateMeshTiltRange(shared_ptr<CrossMesh> crossMesh)
{
    if(crossMesh == nullptr)
        return false;

	int m = crossMesh->crossList.size();
	float big_zero_eps = getVarList()->get<float>("big_zero_eps");
	for(int id = 0; id < m; id++)
	{
		shared_ptr<Cross> cross = crossMesh->crossList[id];
		if(cross == nullptr) continue;
		for(int jd = 0; jd < cross->oriPoints.size(); jd++){
			shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
			if(oriPt == nullptr) continue;
			
			Vector3f t = oriPt->rotation_base; t /= len(t);
			Vector3f e = oriPt->rotation_axis; e /= len(e);
			Vector3f y = oriPt->rotation_base CROSS oriPt->rotation_axis; y /= len(y);
			if(oriPt->tiltSign == 1) y *= -1;
			Vector3f p0 = oriPt->point;
			for(int kd = 0; kd < cross->vers.size(); kd++)
			{
				Vector3f pi = cross->vers[kd].pos;
				double yPiP0 = (y) DOT(pi - p0);
				double tPiP0 = (-t) DOT (pi - p0);
				if(std::abs(tPiP0) < big_zero_eps && std::abs(yPiP0) < big_zero_eps)
				{
					continue;
				}
				if(std::abs(tPiP0) < big_zero_eps && yPiP0 > big_zero_eps)
				{
					oriPt->tilt_range = Vector2f(0, 0);
                    oriPt->sided_range = oriPt->tilt_range;
					continue;
				}
				else{

                    double angle = 0;
				    if(yPiP0 > big_zero_eps){
                        angle = 90 - std::atan(yPiP0/tPiP0) * 180 / M_PI;
				    }
				    else if(std::fabs(yPiP0) <= big_zero_eps){
                        angle = 90;
				    }
				    else{
                        angle = std::fabs(std::atan(yPiP0/tPiP0) * 180 / M_PI) + 90;
				    }
                    if(oriPt->tilt_range.y > angle) oriPt->tilt_range.y = angle;
				}
                oriPt->sided_range = oriPt->tilt_range;
			}
		}
	}

	double min_max = 90;
	double max_min = 0;
	for(int id = 0; id < m; id++){
		shared_ptr<Cross> cross = crossMesh->crossList[id];
		for(int jd = 0; jd < cross->neighbors.size(); jd++)
		{
			shared_ptr<Cross> ncross = cross->neighbors[jd].lock();
			if(ncross == nullptr || cross->crossID > ncross->crossID) continue;
			int edgeID = ncross->GetNeighborEdgeID(cross->crossID);
			shared_ptr<OrientPoint> noriPt = ncross->oriPoints[edgeID];
			shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
			Vector2f range(0, 0);
			range.x = std::max(oriPt->tilt_range.x, noriPt->tilt_range.x);
			range.y = std::min(oriPt->tilt_range.y, noriPt->tilt_range.y);
			oriPt->tilt_range = range;
			noriPt->tilt_range = range;
			//std::cout << "Cross:\t" << cross->crossID << ",\t OriID:\t" << jd << ",\t Range:\t[" << range.x << ", " << range.y << "]" << std::endl;
			if(min_max > range.y) min_max = range.y;
			if(max_min < range.x) max_min = range.x;
		}
	}
	std::cout <<"Range:\t[" << max_min << ",\t" << min_max << "]"<< std::endl;

	if(getVarList()->get<float>("tiltAngle") < max_min || getVarList()->get<float>("tiltAngle") > min_max)
        return false;
	else
        return true;

}

