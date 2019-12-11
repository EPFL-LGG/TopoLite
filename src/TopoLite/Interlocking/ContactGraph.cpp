//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"

#include "Utility/PolyPolyIntersec.h"
#include <iostream>
#include <algorithm>
#include <tbb/tbb.h>


/*************************************************
*
*                  Basic Operation
*
*************************************************/
using pPolyMesh = shared_ptr<PolyMesh>;
ContactGraph::ContactGraph(shared_ptr<InputVarList> varList):TopoObject(varList)
{
}

ContactGraph::~ContactGraph()
{
    nodes.clear();
    edges.clear();
}

void ContactGraph::roundPlane(shared_ptr<_Polygon> poly, int partID, plane_contact &plane)
{
    const float normal2int_scalefactor = 100000;

    Vector3f nrm = poly->ComputeNormal();
    Vector3f center = poly->ComputeCenter();

    plane.nrm = EigenPoint(nrm[0], nrm[1], nrm[2]);


    plane.nx = (int)(nrm[0] * normal2int_scalefactor);
    plane.ny = (int)(nrm[1] * normal2int_scalefactor);
    plane.nz = (int)(nrm[2] * normal2int_scalefactor);

    int reverse = 1;
    if (plane.nx < 0)
        reverse = -1;
    if (plane.nx == 0 && plane.ny < 0)
        reverse = -1;
    if (plane.nx == 0 && plane.ny == 0 && plane.nz < 0)
        reverse = -1;

    nrm *= reverse;
    plane.nx = (int)(nrm[0] * normal2int_scalefactor);
    plane.ny = (int)(nrm[1] * normal2int_scalefactor);
    plane.nz = (int)(nrm[2] * normal2int_scalefactor);

    plane.D = (int)std::floor((nrm ^ center) * normal2int_scalefactor);
    plane.partID = partID;
    plane.polygon = poly;

    return;
}

bool ContactGraph::equalPlane(const plane_contact &A, const plane_contact &B)
{
    if (A.nx == B.nx && A.ny == B.ny && A.nz == B.nz && A.D == B.D){
        return true;
    }
    return false;
}

bool ContactGraph::comparePlane(const plane_contact &A, const plane_contact &B)
{
    if (A.nx < B.nx)
        return true;
    if (A.nx > B.nx)
        return false;
    if (A.ny < B.ny)
        return true;
    if (A.ny > B.ny)
        return false;
    if (A.nz < B.nz)
        return true;
    if (A.nz > B.nz)
        return false;
    if (A.D < B.D)
        return true;
    if (A.D > B.D)
        return false;
    return false;
}

bool ContactGraph::constructFromPolyMeshes(vector<shared_ptr<PolyMesh>> &meshes,
                                           vector<bool> &atBoundary)
{

    //1) create contact planes
    vector<plane_contact> planes;
    for (int id = 0; id < meshes.size(); id++)
    {
        pPolyMesh poly = meshes[id];
        if (poly == nullptr)
            return false;
        for (shared_ptr<_Polygon> face : poly->polyList)
        {
            plane_contact plane;
            roundPlane(face, id, plane);
            planes.push_back(plane);
        }
    }

    std::sort(planes.begin(), planes.end(), comparePlane);

    // for (int id = 0; id < planes.size(); id++)
    // {
    //     std::cout << planes[id].nx
    //               << " " << planes[id].ny
    //               << " " << planes[id].nz
    //               << " " << planes[id].D << std::endl;
    // }

    //2) add nodes
    for(int id = 0; id < meshes.size(); id++){
        meshes[id]->ComputeVolume();
        meshes[id]->ComputeCentroid();
        EigenPoint centroid(meshes[id]->centroid.x, meshes[id]->centroid.y, meshes[id]->centroid.z);
        pContactGraphNode node = make_shared<ContactGraphNode>(atBoundary[id], centroid, centroid, meshes[id]->volume);
        addNode(node);
    }

    int sta = 0, end = 0;
    vector<pairIJ> planeIJ;
    while(sta < planes.size()){
        for(end = sta + 1; end < planes.size(); end++){
            if(!equalPlane(planes[sta], planes[end])){
                break;
            }
        }
        if (end - sta > 1)
        {
            for (int id = sta; id < end; id++)
            {
                for(int jd = id + 1; jd < end; jd++)
                {
                    int partI = planes[id].partID;
                    int partJ = planes[jd].partID;
                    if(partI != partJ)
                        planeIJ.push_back(pairIJ(id, jd));
                }
            }
        }
        sta = end;
    }

    vector<shared_ptr<ContactGraphEdge>> planeIJEdges;
    planeIJEdges.resize(planeIJ.size());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, planeIJ.size()), [&](const tbb::blocked_range<size_t>& r)
	{
        for (size_t id = r.begin(); id != r.end(); ++id)
		{
            int planeI = planeIJ[id].first;
            int planeJ = planeIJ[id].second;
            
            double projMat[16];
            double invsProjMat[16];

            pPolygon polyI = planes[planeI].polygon.lock();
            pPolygon polyJ = planes[planeJ].polygon.lock();

            polyI->ComputeProjectMatrixTo2D(projMat, invsProjMat);
            vector<Vector3f> polyI2D = polyI->ProjectPolygonTo2D(projMat);
            vector<Vector3f> polyJ2D = polyJ->ProjectPolygonTo2D(projMat);
            vector<Vector3f> overlapPolyPts;

            PolyPolyIntersec ppIntersec(getVarList());

            ppIntersec.ComputePolygonsIntersection(polyI2D, polyJ2D, overlapPolyPts);
            if (overlapPolyPts.size() == 0)
                continue;

            vector<Vector3f> contactPoly;
            ppIntersec.ProjectPolygonTo3D(overlapPolyPts, invsProjMat, contactPoly);

            if(!contactPoly.empty() && contactPoly.size() >= 3){
                int partI = planes[planeI].partID;
                int partJ = planes[planeJ].partID;

                ContactPolygon contactPolyEigen;
                EigenPoint ct(0, 0, 0);
                for(Vector3f pt: contactPoly){
                    EigenPoint ptEigen(pt[0], pt[1], pt[2]);
                    contactPolyEigen.points.push_back(ptEigen);
                    ct += ptEigen;
                }
                contactPolyEigen.center = ct/contactPoly.size();

                shared_ptr<ContactGraphEdge> edge = make_shared<ContactGraphEdge>(contactPolyEigen, planes[planeI].nrm);
                planeIJEdges[id] = edge;
            }
            else{
                planeIJEdges[id] = nullptr;
            }
        }
    });

    for(int id = 0; id < planeIJ.size(); id++)
    {
        pContactGraphEdge edge = planeIJEdges[id];
        if(edge != nullptr){
            int planeI = planeIJ[id].first;
            int planeJ = planeIJ[id].second;
            int partI = planes[planeI].partID;
            int partJ = planes[planeJ].partID;
            addContact(nodes[partI], nodes[partJ], edge);
        }
    }

    return true;
}

/*************************************************
*
*                  Graph Operation
*
*************************************************/

void ContactGraph::addNode(shared_ptr<ContactGraphNode> _node)
{
    _node->staticID = nodes.size();

    nodes.push_back(_node);

    return;
}

void ContactGraph::addContact(shared_ptr<ContactGraphNode> _nodeA, shared_ptr<ContactGraphNode> _nodeB, shared_ptr<ContactGraphEdge> _edge)
{

    if (_nodeA->isBoundary && _nodeB->isBoundary)
        return;

    _edge->partIDA = _nodeA->staticID;
    _edge->partIDB = _nodeB->staticID;

    edges.push_back(_edge);

    pair<wpContactGraphNode, wpContactGraphEdge> contactNeighbor;

    contactNeighbor.first = _nodeB;
    contactNeighbor.second = _edge;
    _nodeA->neighbors.push_back(contactNeighbor);

    contactNeighbor.first = _nodeA;
    contactNeighbor.second = _edge;
    _nodeB->neighbors.push_back(contactNeighbor);

    return;
}

void ContactGraph::finalize()
{
    int dynamicID = 0;
    dynamic_nodes.clear();
    for (shared_ptr<ContactGraphNode> node : nodes)
    {
        if (!node->isBoundary)
        {
            node->dynamicID = dynamicID++;
            dynamic_nodes.push_back(node);
        }
        else
        {
            node->dynamicID = -1;
        }
    }
}
