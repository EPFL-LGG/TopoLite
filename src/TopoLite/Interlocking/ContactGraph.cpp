//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"

#include "Utility/PolyPolyBoolean.h"
#include <iostream>
#include <algorithm>
#include <set>
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

bool ContactGraph::constructFromPolyMeshes(vector<shared_ptr<PolyMesh>> &meshes,
                                           vector<bool> &atBoundary,
                                           double eps)
{

    //1) create contact planes
    vector<plane_contact> planes;
    std::set<plane_contact, plane_contact_compare> setPlanes;

    int groupID = 0;
    for (int id = 0; id < meshes.size(); id++)
    {
        pPolyMesh poly = meshes[id];
        if (poly == nullptr)
            return false;
        for (shared_ptr<_Polygon> face : poly->polyList)
        {
            if(face->vers.size() < 3) continue;

            //1.1) construct plane
            plane_contact plane;
            Vector3f nrm = face->ComputeNormal();
            Vector3f center = face->vers[0].pos;
            plane.nrm = EigenPoint(nrm[0], nrm[1], nrm[2]);
            
            plane.D = nrm ^ center;
            plane.partID = id;
            plane.polygon = face;
            plane.eps = eps;

            //1.2) find groupID
            std::set<plane_contact, plane_contact_compare>::iterator find_it = setPlanes.end();
            for(int reverse = -1; reverse <= 1; reverse += 2){
                plane_contact tmp_plane = plane;
                tmp_plane.nrm *= reverse;
                tmp_plane.D *= reverse;
                find_it = setPlanes.find(tmp_plane);
                if(find_it != setPlanes.end()){
                    plane.groupID = (*find_it).groupID;
                    break;
                }
            }

            if(find_it == setPlanes.end()){
                plane.groupID = groupID ++;
                setPlanes.insert(plane);
            }
            
            planes.push_back(plane);
        }
    }

    std::sort(planes.begin(), planes.end(), [&](const plane_contact &A, const plane_contact &B){
        return A.groupID < B.groupID;
    });

     //2) add nodes
     for(int id = 0; id < meshes.size(); id++){
         meshes[id]->ComputeVolume();
         meshes[id]->ComputeCentroid();
         EigenPoint centroid(meshes[id]->centroid.x, meshes[id]->centroid.y, meshes[id]->centroid.z);
         pContactGraphNode node = make_shared<ContactGraphNode>(atBoundary[id], centroid, centroid, meshes[id]->volume);
         addNode(node);
     }

     //3) find all pairs of contact polygon
     int sta = 0, end = 0;
     vector<pairIJ> planeIJ;
     while(sta < planes.size())
     {
          for(end = sta + 1; end < planes.size(); end++)
          {
              if(planes[sta].groupID != planes[end].groupID)
              {
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

                      EigenPoint nrmI = planes[id].nrm.normalized();
                      EigenPoint nrmJ = planes[jd].nrm.normalized();

                      if(partI != partJ
                      && std::abs(nrmI.dot(nrmJ) + 1) < FLOAT_ERROR_LARGE
                      && (!atBoundary[partI] || !atBoundary[partJ]))
                          planeIJ.push_back(pairIJ(id, jd));
                  }
              }
          }
          sta = end;
      }


     //4) parallel compute contacts
     vector<shared_ptr<ContactGraphEdge>> planeIJEdges;
     planeIJEdges.resize(planeIJ.size());

     tbb::parallel_for(tbb::blocked_range<size_t>(0, planeIJ.size()), [&](const tbb::blocked_range<size_t>& r)
	 {
         for (size_t id = r.begin(); id != r.end(); ++id)
         //for (size_t id = 0; id != planeIJ.size(); ++id)
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
             vector<vector<Vector3f>> overlapPolyPts;

             PolyPolyBoolean ppIntersec(getVarList());

             ppIntersec.ComputePolygonsIntersection(polyI2D, polyJ2D, overlapPolyPts);
             if (overlapPolyPts.size() == 0)
                 continue;

             vector<ContactPolygon> contactPolyEigens;
             for(vector<Vector3f> overlapPoly: overlapPolyPts){
                 vector<Vector3f> contactPoly;
                 ppIntersec.ProjectPolygonTo3D(overlapPoly, invsProjMat, contactPoly);
                 if(contactPoly.size() >= 3){
                     ContactPolygon contactPolyEigen;
                     EigenPoint ct(0, 0, 0);
                     for(Vector3f pt: contactPoly)
                     {
                        EigenPoint ptEigen(pt[0], pt[1], pt[2]);
                        contactPolyEigen.points.push_back(ptEigen);
                        ct += ptEigen;
                     }
                     contactPolyEigen.center = ct/contactPoly.size();
                     contactPolyEigens.push_back(contactPolyEigen);
                 }
             }

             if(!contactPolyEigens.empty()){
                 int partI = planes[planeI].partID;
                 int partJ = planes[planeJ].partID;

                 shared_ptr<ContactGraphEdge> edge = make_shared<ContactGraphEdge>(contactPolyEigens, planes[planeI].nrm);
                 planeIJEdges[id] = edge;
             }
         }
     });

     //5) add contact edges
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

void ContactGraph::getContactMesh(pPolyMesh &mesh)
{
    mesh.reset();
    mesh = make_shared<PolyMesh>(getVarList());
    for(pContactGraphEdge edge: edges)
    {
        for(ContactPolygon poly: edge->polygons)
        {
            shared_ptr<_Polygon> face = make_shared<_Polygon>();
            for(EigenPoint pt: poly.points)
            {
                face->push_back(Vector3f(pt[0], pt[1], pt[2]));
            }
            mesh->polyList.push_back(face);
        }
    }
    if(mesh)
        mesh->removeDuplicatedVertices();
}