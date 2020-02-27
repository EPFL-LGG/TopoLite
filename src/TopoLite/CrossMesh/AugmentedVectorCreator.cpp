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

#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "AugmentedVectorCreator.h"


//**************************************************************************************//
//                                   Standalone functions
//**************************************************************************************//

void FindARoot(const pCrossMesh &crossMesh, shared_ptr<Cross> &root);

/*
 *  Find a root for Breadth-First Search Traversal
 */
void FindARoot(const pCrossMesh &crossMesh, shared_ptr<Cross> &root) {
    for (const auto &item : crossMesh->crossList) {
        if (!item->atBoundary) {
            root = item;
            return;
        }
    }
}

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

AugmentedVectorCreator::~AugmentedVectorCreator() {

}


//**************************************************************************************//
//                           Convert Polygonal Mesh into Cross Mesh
//**************************************************************************************//

void AugmentedVectorCreator::CreateAugmentedVector(float tiltAngle, pCrossMesh &crossMesh) {
    if (crossMesh) {
        InitMeshTiltNormals(crossMesh);
        InitMeshTiltNormalsResolveConflicts(crossMesh, tiltAngle);
    }
}


void AugmentedVectorCreator::InitMeshTiltNormals(const pCrossMesh& crossMesh) {
    for (const auto &cross : crossMesh->crossList) {
        cross->InitTiltNormals();
    }
}


void AugmentedVectorCreator::InitMeshTiltNormalsResolveConflicts(const pCrossMesh &crossMesh, float tiltAngle) {
    vector<pCross> bfsQueue;

    if (crossMesh->crossList.empty())
        return;

    // Breadth-First Search Traversal
    //
    // Ref: https://en.wikipedia.org/wiki/Breadth-first_search

    // Start at tree root (or is it an arbitrary root here?)

    shared_ptr<Cross> root;

    FindARoot(crossMesh, root);

    if (root == nullptr)
        return;

    root->UpdateTiltNormal_Root(tiltAngle);
    root->isVisited = true;
    bfsQueue.push_back(root);


    // Explores all the neighbor nodes at present depth before going to next depth level

    while (!bfsQueue.empty()) {
        shared_ptr<Cross> currVisit = bfsQueue[0];
        bfsQueue.erase(bfsQueue.begin());


        bool isAllVisited = true;

        for (const auto &cross : crossMesh->crossList) {
            if (cross == nullptr)
                continue;
            if (!cross->isVisited) {
                isAllVisited = false;
                break;
            }
        }

        // Quit if all crosses have been visited

        if (isAllVisited)
            break;

        // Process the neighbors of current cross

        for (int i = 0; i < currVisit->neighbors.size(); i++) {
//			printf(" i=%d \n", i);
            shared_ptr<Cross> nextVisit = currVisit->neighbors[i].lock();

            if (    nextVisit == nullptr
                ||  nextVisit->isVisited
                || (nextVisit->atBoundary && getVarList()->get<bool>("ground_touch_bdry")))
                continue;

            nextVisit->UpdateTiltNormal(tiltAngle);
            nextVisit->isVisited = true;
            bfsQueue.push_back(nextVisit);
        }
    }

    if (getVarList()->get<bool>("ground_touch_bdry")) {
        for (auto &id : crossMesh->crossList) {
            if (id->atBoundary)
                id->UpdateTiltNormal(tiltAngle);
        }
    }
}


void AugmentedVectorCreator::UpdateMeshTiltNormals(const pCrossMesh &crossMesh, float tiltAngle) {

    if (crossMesh == nullptr)
        return;

    for (const auto &cross : crossMesh->crossList) {
        // cross is a shared_ptr
        if (cross == nullptr)
            continue;

        for (int jd = 0; jd < cross->oriPoints.size(); jd++) {
            shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
            if (cross->neighbors[jd].lock() != nullptr) {
                if (!cross->neighbors[jd].lock()->atBoundary || !cross->atBoundary) {
                    oriPt->normal = cross->RotateNormal(oriPt->rotation_base,
                                                        oriPt->rotation_axis,
                                                        tiltAngle * oriPt->tiltSign);
                    oriPt->update_rotation(tiltAngle);
                }
            }
        }
    }
}


bool AugmentedVectorCreator::UpdateMeshTiltRange(const shared_ptr<CrossMesh> &crossMesh) {
    if (crossMesh == nullptr)
        return false;

    auto big_zero_eps = getVarList()->get<float>("big_zero_eps");
    for (const auto &cross : crossMesh->crossList) {
        if (cross == nullptr)
            continue;

        for (int jd = 0; jd < cross->oriPoints.size(); jd++) {
            shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
            if (oriPt == nullptr)
                continue;

            Vector3f t = oriPt->rotation_base;
            t /= len(t);
            Vector3f e = oriPt->rotation_axis;
            e /= len(e);
            Vector3f y = oriPt->rotation_base CROSS oriPt->rotation_axis;
            y /= len(y);

            if (oriPt->tiltSign == 1)
                y *= -1;

            Vector3f p0 = oriPt->point;

            for (auto  &ver : cross->vers) {
                Vector3f pi = ver.pos;
                double yPiP0 = (y) DOT (pi - p0);
                double tPiP0 = (-t) DOT (pi - p0);
                if (std::abs(tPiP0) < big_zero_eps && std::abs(yPiP0) < big_zero_eps)
                    continue;

                if (std::abs(tPiP0) < big_zero_eps && yPiP0 > big_zero_eps) {
                    oriPt->tilt_range = Vector2f(0, 0);
                    oriPt->sided_range = oriPt->tilt_range;
                    continue;
                } else {

                    double angle = 0;
                    if (yPiP0 > big_zero_eps) {
                        angle = 90 - std::atan(yPiP0 / tPiP0) * 180 / M_PI;
                    } else if (std::fabs(yPiP0) <= big_zero_eps) {
                        angle = 90;
                    } else {
                        angle = std::fabs(std::atan(yPiP0 / tPiP0) * 180 / M_PI) + 90;
                    }
                    if (oriPt->tilt_range.y > angle) oriPt->tilt_range.y = angle;
                }
                oriPt->sided_range = oriPt->tilt_range;
            }
        }
    }

    double min_max = 90.0;
    double max_min = 0.0;

    for (const auto &cross : crossMesh->crossList) {
        for (int jd = 0; jd < cross->neighbors.size(); jd++) {
            shared_ptr<Cross> ncross = cross->neighbors[jd].lock();

            if (ncross == nullptr || cross->crossID > ncross->crossID)
                continue;

            int edgeID = ncross->GetNeighborEdgeID(cross->crossID);
            shared_ptr<OrientPoint> noriPt = ncross->oriPoints[edgeID];
            shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
            Vector2f range(0, 0);
            range.x = std::max(oriPt->tilt_range.x, noriPt->tilt_range.x);
            range.y = std::min(oriPt->tilt_range.y, noriPt->tilt_range.y);

            // FIXME: 2 unused variables
            oriPt->tilt_range = range;
            noriPt->tilt_range = range;
//            std::cout << "Cross:\t" << cross->crossID << ",\t OriID:\t" << jd << ",\t Range:\t[" << range.x << ", " << range.y << "]" << std::endl;
            if (min_max > range.y) min_max = range.y;
            if (max_min < range.x) max_min = range.x;
        }
    }
    std::cout << "Range:\t[" << max_min << ",\t" << min_max << "]" << std::endl;

    auto tiltangle = getVarList()->get<float>("tiltAngle");
    return tiltangle < max_min || tiltangle > min_max ? false : true;
}