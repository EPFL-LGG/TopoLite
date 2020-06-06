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

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//
#include "AugmentedVectorCreator.h"

template<typename Scalar>
AugmentedVectorCreator<Scalar>::~AugmentedVectorCreator() {

}

//**************************************************************************************//
//
//**************************************************************************************//

template<typename Scalar>
void AugmentedVectorCreator<Scalar>::createAugmentedVectors(Scalar tiltAngle, pCrossMesh crossMesh) {
    if (crossMesh) {
        InitMeshTiltNormals(crossMesh);
        InitMeshTiltNormalsResolveConflicts(crossMesh, tiltAngle);
    }
}

template<typename Scalar>
void AugmentedVectorCreator<Scalar>::updateAugmentedVectors(Scalar tiltAngle, pCrossMesh crossMesh) {

    if (crossMesh == nullptr)
        return;

    for (size_t id = 0; id < crossMesh->size(); id++) {
        pCross cross = crossMesh->cross(id);
        // cross is a shared_ptr
        if (cross == nullptr)
            continue;

        for (int jd = 0; jd < cross->oriPoints.size(); jd++) {
            shared_ptr<OrientPoint<Scalar>> oriPt = cross->oriPoints[jd];
            if (cross->neighbors[jd].lock() != nullptr) {
                if (!cross->neighbors[jd].lock()->atBoundary || !cross->atBoundary) {
                    oriPt->updateAngle(tiltAngle);
                }
            }
        }
    }
}

//**************************************************************************************//
//
//**************************************************************************************//

template<typename Scalar>
void AugmentedVectorCreator<Scalar>::InitMeshTiltNormals(pCrossMesh crossMesh)
{
    tbb::parallel_for(tbb::blocked_range<size_t>(0, crossMesh->size()), [&](const tbb::blocked_range<size_t> &r){
        for(size_t id = r.begin(); id != r.end(); ++id){
            crossMesh->cross(id)->initTiltNormals();
        }
    });
}


template<typename Scalar>
void AugmentedVectorCreator<Scalar>::InitMeshTiltNormalsResolveConflicts(pCrossMesh crossMesh, Scalar tiltAngle) {

    if (crossMesh->size() == 0)
        return;

    // Start at any non-boundary cross
    wpCross nonboundary_cross;
    for(size_t id = 0; id < crossMesh->size(); ++id){
        if(crossMesh->cross(id)->atBoundary == false){
            nonboundary_cross = crossMesh->cross(id);
            break;
        }
    }
    //if every cross is at boundary, exist
    if (nonboundary_cross.lock() == nullptr)
        return;

    // Breadth-First Search Traversal
    // Ref: https://en.wikipedia.org/wiki/Breadth-first_search
    std::queue<wpCross> bfsQueue;
    std::unordered_map<Cross<Scalar> *, bool> crossVisited;
    nonboundary_cross.lock()->updateTiltNormalsRoot(tiltAngle);

    bfsQueue.push(nonboundary_cross.lock());
    crossVisited[nonboundary_cross.lock().get()] = true;


    // Explores all the neighbor nodes at present depth before going to next depth level
    // here, if user select "ground_touch_brdy", we only update non-boundary cross
    while (!bfsQueue.empty())
    {
        wpCross currVisit = bfsQueue.front();
        bfsQueue.pop();

        // Process the neighbors of current cross
        for (wpCross nextVisit: currVisit.lock()->neighbors)
        {
            if (    nextVisit.lock() == nullptr
                ||  crossVisited.find(nextVisit.lock().get()) != crossVisited.end()
                || (nextVisit.lock()->atBoundary && getVarList()->getBool("ground_touch_bdry")))
                continue;

            nextVisit.lock()->updateTiltNormals(tiltAngle, crossVisited);
            crossVisited[nextVisit.lock().get()] = true;
            bfsQueue.push(nextVisit.lock());
        }
    }

    //update the reset of boundary crosses
    if (getVarList()->getBool("ground_touch_bdry")) {
        for (size_t id = 0; id < crossMesh->size(); ++id) {
            wpCross part = crossMesh->cross(id);
            if (part.lock()->atBoundary){
                part.lock()->updateTiltNormals(tiltAngle, crossVisited);
                crossVisited[part.lock().get()] = true;
            }
        }
    }
}

template<typename Scalar>
bool AugmentedVectorCreator<Scalar>::UpdateMeshTiltRange(pCrossMesh crossMesh) {
    if (crossMesh == nullptr)
        return false;

    auto big_zero_eps = getVarList()->getFloat("big_zero_eps");
    for (int id = 0; id < crossMesh->size(); id++)
    {
        pCross cross = crossMesh->cross(id);
        if (cross == nullptr)
            continue;

        for (int jd = 0; jd < cross->oriPoints.size(); jd++)
        {
            shared_ptr<OrientPoint<Scalar>> oriPt = cross->oriPoints[jd];
            if (oriPt == nullptr)
                continue;

            Vector3 t = oriPt->rotation_base.normalized();
            
            Vector3 y = (oriPt->rotation_base.cross(oriPt->rotation_axis)).normalized();
            if (oriPt->tiltSign == 1) y *= -1;

            Vector3 p0 = oriPt->point;

            for (pVertex ver : cross->vers) {
                Vector3 pi = ver->pos;
                double yPiP0 = (y).dot(pi - p0);
                double tPiP0 = (-t).dot(pi - p0);
                if (std::abs(tPiP0) < big_zero_eps && std::abs(yPiP0) < big_zero_eps)
                    continue;

                if (std::abs(tPiP0) < big_zero_eps && yPiP0 > big_zero_eps) {
                    oriPt->tilt_range = Vector2(0, 0);
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
                    if (oriPt->tilt_range.y() > angle) oriPt->tilt_range.y() = angle;
                }
                oriPt->sided_range = oriPt->tilt_range;
            }
        }
    }

    double min_max = 90.0;
    double max_min = 0.0;

    for (int id = 0; id < crossMesh->size(); id++)
    {
        pCross cross = crossMesh->cross(id);
        for (int jd = 0; jd < cross->neighbors.size(); jd++)
        {
            shared_ptr<Cross<Scalar>> ncross = cross->neighbors[jd].lock();

            if (ncross == nullptr || cross->crossID > ncross->crossID)
                continue;

            int edgeID = ncross->getEdgeIDSharedWithCross(cross.get());
            shared_ptr<OrientPoint<Scalar>> noriPt = ncross->oriPoints[edgeID];
            shared_ptr<OrientPoint<Scalar>> oriPt = cross->oriPoints[jd];
            Vector2 range(0, 0);
            range.x() = std::max(oriPt->tilt_range.x(), noriPt->tilt_range.x());
            range.y() = std::min(oriPt->tilt_range.y(), noriPt->tilt_range.y());

            // FIXME: 2 unused variables
            oriPt->tilt_range = range;
            noriPt->tilt_range = range;
//            std::cout << "Cross:\t" << cross->crossID << ",\t OriID:\t" << jd << ",\t Range:\t[" << range.x << ", " << range.y << "]" << std::endl;
            if (min_max > range.y()) min_max = range.y();
            if (max_min < range.x()) max_min = range.x();
        }
    }
    std::cout << "Range:\t[" << max_min << ",\t" << min_max << "]" << std::endl;

    Scalar tiltangle = getVarList()->getFloat("tiltAngle");
    return tiltangle < max_min || tiltangle > min_max ? false : true;
}

template class AugmentedVectorCreator<double>;
