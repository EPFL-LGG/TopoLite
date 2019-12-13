//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraph.h"

#include "Utility/PolyPolyBoolean.h"
#include <iostream>
#include <algorithm>
#include <set>
#include <tbb/tbb.h>
//
#ifndef CATCH2_UNITTEST
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

void ContactGraph::mergeFacesPolyMesh(vector<shared_ptr<PolyMesh>> &meshes, double eps)
{
    for(int id = 0; id < meshes.size(); id++)
    {
        pPolyMesh mesh = meshes[id];
        vector<plane_contact> planes;
        std::set<plane_contact, plane_contact_compare> setPlanes;

        int groupID = 0;
        for (shared_ptr<_Polygon> face : mesh->polyList)
        {

            //1.1) construct plane
            plane_contact plane;
            Vector3f nrm = face->ComputeNormal();
            Vector3f center = face->ComputeCenter();
            plane.nrm = EigenPoint(nrm[0], nrm[1], nrm[2]);

            plane.D = nrm ^ center;
            plane.partID = id;
            plane.polygon = face;
            plane.eps = eps;

            //1.2) find groupID
            auto find_it = setPlanes.find(plane);

            if(find_it == setPlanes.end()){
                plane.groupID = groupID ++;
                setPlanes.insert(plane);
            }
            else{
                plane.groupID = find_it->groupID;
            }

            planes.push_back(plane);
        }

        std::sort(planes.begin(), planes.end(), [&](const plane_contact &A, const plane_contact &B){
            return A.groupID < B.groupID;
        });

        int sta = 0, end = 0;
        vector<shared_ptr<_Polygon>> polygons;
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
                vector<vector<Vector3f>> allFaces;
                for (int kd = sta; kd < end; kd++)
                {
                    if(planes[kd].polygon.lock())
                        allFaces.push_back(planes[kd].polygon.lock()->GetVertices());
                }
                vector<vector<Vector3f>> mergeFaces;
                PolyPolyBoolean polyBoolean(getVarList());
                polyBoolean.ComputePolygonsUnion(allFaces, mergeFaces);
                for(int kd = 0; kd < mergeFaces.size(); kd++){
                    shared_ptr<_Polygon> poly = make_shared<_Polygon>();
                    poly->SetVertices(mergeFaces[kd]);
                    polygons.push_back(poly);
                }
            }
            else{
                polygons.push_back(planes[sta].polygon.lock());
            }
            sta = end;
        }

        meshes[id]->polyList = polygons;
        meshes[id]->UpdateVertices();
    }

    return;
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

//     tbb::parallel_for(tbb::blocked_range<size_t>(0, planeIJ.size()), [&](const tbb::blocked_range<size_t>& r)
//	 {
         for (size_t id = 0; id != planeIJ.size(); ++id)
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
//     });

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
}

#else

#include <catch2/catch.hpp>
#include "Interlocking/ContactGraph.h"
#include "Mesh/PolyMesh.h"
#include "IO/XMLIO.h"
using pPolyMesh = shared_ptr<PolyMesh>;
using pPolygon = shared_ptr<_Polygon>;


TEST_CASE("Class ContactGraph")
{
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());


    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,-1]")
    {
        shared_ptr<_Polygon> pA = make_shared<_Polygon>();
        pA->push_back(Vector3f(0, 0, 0));
        pA->push_back(Vector3f(2, 0, 0));
        pA->push_back(Vector3f(2, 2, 0));
        pA->push_back(Vector3f(0, 2, 0));

        shared_ptr<_Polygon> pB = make_shared<_Polygon>();
        pB->push_back(Vector3f(1, 1, 0));
        pB->push_back(Vector3f(1, 3, 0));
        pB->push_back(Vector3f(3, 3, 0));
        pB->push_back(Vector3f(3, 1, 0));

        pPolyMesh meshA = make_shared<PolyMesh>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon contactPolygon;
        for(EigenPoint pt: graph->edges[0]->polygons[0].points){
           contactPolygon.push_back(Vector3f(pt[0], pt[1], pt[2]));
        }
        REQUIRE(std::abs(contactPolygon.ComputeArea() - 1) < FLOAT_ERROR_LARGE);
    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,1]")
    {
        shared_ptr<_Polygon> pA = make_shared<_Polygon>();
        pA->push_back(Vector3f(0, 0, 0));
        pA->push_back(Vector3f(2, 0, 0));
        pA->push_back(Vector3f(2, 2, 0));
        pA->push_back(Vector3f(0, 2, 0));

        shared_ptr<_Polygon> pB = make_shared<_Polygon>();
        pB->push_back(Vector3f(1, 1, 0));
        pB->push_back(Vector3f(3, 1, 0));
        pB->push_back(Vector3f(3, 3, 0));
        pB->push_back(Vector3f(1, 3, 0));

        pPolyMesh meshA = make_shared<PolyMesh>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 0);

    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,1] but centor at Z:-0.0004")
    {
        shared_ptr<_Polygon> pA = make_shared<_Polygon>();
        pA->push_back(Vector3f(0, 0, 0));
        pA->push_back(Vector3f(2, 0, 0));
        pA->push_back(Vector3f(2, 2, 0));
        pA->push_back(Vector3f(0, 2, 0));

        shared_ptr<_Polygon> pB = make_shared<_Polygon>();
        pB->push_back(Vector3f(1, 1, -0.0004));
        pB->push_back(Vector3f(1, 3, -0.0004));
        pB->push_back(Vector3f(3, 3, -0.0004));
        pB->push_back(Vector3f(3, 1, -0.0004));

        pPolyMesh meshA = make_shared<PolyMesh>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon contactPolygon;
        if(graph->edges.size() > 0){
            for(EigenPoint pt: graph->edges[0]->polygons[0].points){
                contactPolygon.push_back(Vector3f(pt[0], pt[1], pt[2]));
            }
            REQUIRE(std::abs(contactPolygon.ComputeArea() - 1) < FLOAT_ERROR_LARGE);
        }
    }

    SECTION("both A B at boundary")
    {
        shared_ptr<_Polygon> pA = make_shared<_Polygon>();
        pA->push_back(Vector3f(0, 0, 0));
        pA->push_back(Vector3f(2, 0, 0));
        pA->push_back(Vector3f(2, 2, 0));
        pA->push_back(Vector3f(0, 2, 0));

        shared_ptr<_Polygon> pB = make_shared<_Polygon>();
        pB->push_back(Vector3f(1, 1, 0.0004));
        pB->push_back(Vector3f(1, 3, 0.0004));
        pB->push_back(Vector3f(3, 3, 0.0004));
        pB->push_back(Vector3f(3, 1, 0.0004));

        pPolyMesh meshA = make_shared<PolyMesh>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(true);
        atBoundary.push_back(true);

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 0);

    }

//    SECTION("read xml file and compare contact numbers"){
//        XMLIO Reader;
//        XMLData data;
//
//        boost::filesystem::path current_path(boost::filesystem::current_path());
//        boost::filesystem::path debugxml_filepath;
//        if(current_path.filename() == "TopoLite"){
//            debugxml_filepath = current_path / "data/origin.xml";
//        }
//        else{
//            debugxml_filepath = current_path / "../data/origin.xml";
//        }
//
//        REQUIRE(Reader.XMLReader(debugxml_filepath.string(), data) == 1);
//
//        shared_ptr<Struc> struc = data.strucCreator->struc;
//        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(data.varList);
//
//        vector<shared_ptr<PolyMesh>> meshes;
//        vector<bool> atBoundary;
//
//        for(int partID = 0; partID < struc->partList.size(); partID++){
//            pPart part = struc->partList[partID];
//            meshes.push_back(part->polyMesh);
//            atBoundary.push_back(part->atBoundary);
//        }
//        graph->constructFromPolyMeshes(meshes, atBoundary);
//
//        struc->contactList.clear();
//        struc->innerContactList.clear();
//        struc->ComputePartFaceContacts();
//
//        REQUIRE(graph->edges.size() == struc->innerContactList.size());
//    }

    SECTION("merge all face"){
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());
        vector<shared_ptr<PolyMesh>> meshes;
        pPolyMesh mesh = make_shared<PolyMesh>(varList);

        boost::filesystem::path current_path(boost::filesystem::current_path());
        boost::filesystem::path objfilepath;
        if(current_path.filename() == "TopoLite"){
            objfilepath = current_path / "data/origin_data/PartGeometry/Part_01.obj";
        }
        else{
            objfilepath = current_path / "../data/origin_data/PartGeometry/Part_01.obj";
        }

        bool texture;
        mesh->ReadOBJModel(objfilepath.string().c_str(), texture, false);
        meshes.push_back(mesh);

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);

        graph->mergeFacesPolyMesh(meshes);

        REQUIRE(meshes[0]->polyList.size() == 8);
    }

    SECTION("construct from file"){
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());
        vector<shared_ptr<PolyMesh>> meshes;

        vector<bool> atBoundary;
        int partID[9] = {0, 1};
        for (int id = 0; id < 2; id++)
        {
            pPolyMesh mesh = make_shared<PolyMesh>(varList);
            boost::filesystem::path current_path(boost::filesystem::current_path());
            boost::filesystem::path objfilepath;
            boost::filesystem::path filename = string("puz/") + std::to_string(partID[id]) + ".obj";
            if (current_path.filename() == "TopoLite")
            {
                objfilepath = current_path / "data" / filename;
            }
            else
            {
               objfilepath = current_path.parent_path() / "data" / filename;
            }

            bool texture;
            REQUIRE(mesh->ReadOBJModel(objfilepath.c_str(), texture, false) == true);
            meshes.push_back(mesh);
            atBoundary.push_back(false);
        }

        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(varList);

        graph->mergeFacesPolyMesh(meshes);

        graph->constructFromPolyMeshes(meshes, atBoundary);

        pPolyMesh contact_mesh;
        graph->getContactMesh(contact_mesh);
        if(contact_mesh)
        {
            boost::filesystem::path current_path(boost::filesystem::current_path());
            boost::filesystem::path objfilepath;
            boost::filesystem::path filename = string("puz/generated.obj");
            if (current_path.filename() == "TopoLite")
            {
                objfilepath = current_path / "data" / filename;
            }
            else
            {
               objfilepath = current_path / "../data" / filename;
            }
            contact_mesh->WriteOBJModel(objfilepath.c_str());
        }
      
    }
}

#endif