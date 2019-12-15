//
// Created by ziqwang on 14.01.19.
//

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

    SECTION("read xml file and compare contact numbers"){
        XMLIO Reader;
        XMLData data;

        boost::filesystem::path current_path(boost::filesystem::current_path());
        boost::filesystem::path debugxml_filepath;
        if(current_path.filename() == "TopoLite"){
            debugxml_filepath = current_path / "data/TopoInterock/XML/origin.xml";
        }
        else{
            debugxml_filepath = current_path / "../data/TopoInterock/XML/origin.xml";
        }

        REQUIRE(Reader.XMLReader(debugxml_filepath.string(), data) == 1);

        shared_ptr<Struc> struc = data.strucCreator->struc;
        shared_ptr<ContactGraph> graph = make_shared<ContactGraph>(data.varList);

        vector<shared_ptr<PolyMesh>> meshes;
        vector<bool> atBoundary;

        for(int partID = 0; partID < struc->partList.size(); partID++){
            pPart part = struc->partList[partID];
            meshes.push_back(part->polyMesh);
            atBoundary.push_back(part->atBoundary);
        }
        graph->constructFromPolyMeshes(meshes, atBoundary);

        struc->contactList.clear();
        struc->innerContactList.clear();
        struc->ComputePartFaceContacts();

        REQUIRE(graph->edges.size() == struc->innerContactList.size());
    }


}