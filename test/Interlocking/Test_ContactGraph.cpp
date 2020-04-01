//
// Created by ziqwang on 14.01.19.
//

#include <catch2/catch.hpp>
#include "Interlocking/ContactGraph.h"
#include "Mesh/PolyMesh.h"

using pPolyMesh = shared_ptr<PolyMesh<double>>;
using pPolygon = shared_ptr<_Polygon<double>>;
using Eigen::Vector3d;

TEST_CASE("Class ContactGraph")
{
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,-1]")
    {
        pPolygon pA = make_shared<_Polygon<double>>();
        pA->push_back(Vector3d(0, 0, 0));
        pA->push_back(Vector3d(2, 0, 0));
        pA->push_back(Vector3d(2, 2, 0));
        pA->push_back(Vector3d(0, 2, 0));

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(1, 3, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(3, 1, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon<double> contactPolygon;
        for(const auto& ver: graph->edges[0]->polygons[0]->vers){
            contactPolygon.push_back(Vector3d(ver->pos[0], ver->pos[1], ver->pos[2]));
        }
        REQUIRE(std::abs(contactPolygon.area() - 1) < 1e-4);
    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,1]")
    {
        pPolygon pA = make_shared<_Polygon<double>>();
        pA->push_back(Vector3d(0, 0, 0));
        pA->push_back(Vector3d(2, 0, 0));
        pA->push_back(Vector3d(2, 2, 0));
        pA->push_back(Vector3d(0, 2, 0));

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(3, 1, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(1, 3, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 0);

    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,1] but centor at Z:-0.0004")
    {
        pPolygon pA = make_shared<_Polygon<double>>();
        pA->push_back(Vector3d(0, 0, 0));
        pA->push_back(Vector3d(2, 0, 0));
        pA->push_back(Vector3d(2, 2, 0));
        pA->push_back(Vector3d(0, 2, 0));

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, -0.0004));
        pB->push_back(Vector3d(1, 3, -0.0004));
        pB->push_back(Vector3d(3, 3, -0.0004));
        pB->push_back(Vector3d(3, 1, -0.0004));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon<double> contactPolygon;
        if(graph->edges.size() > 0){
            for(auto ver: graph->edges[0]->polygons[0]->vers){
                contactPolygon.push_back(Vector3d(ver->pos[0], ver->pos[1], ver->pos[2]));
            }
            REQUIRE(std::abs(contactPolygon.area() - 1) < 1e-4);
        }
    }

    SECTION("both A B at boundary")
    {
        pPolygon pA = make_shared<_Polygon<double>>();
        pA->push_back(Vector3d(0, 0, 0));
        pA->push_back(Vector3d(2, 0, 0));
        pA->push_back(Vector3d(2, 2, 0));
        pA->push_back(Vector3d(0, 2, 0));

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0.0004));
        pB->push_back(Vector3d(1, 3, 0.0004));
        pB->push_back(Vector3d(3, 3, 0.0004));
        pB->push_back(Vector3d(3, 1, 0.0004));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(true);
        atBoundary.push_back(true);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 0);
    }


    SECTION("Scaling Error"){
        pPolyMesh A, B;
        A = make_shared<PolyMesh<double>>(varList);
        B = make_shared<PolyMesh<double>>(varList);

        string strA = "../data/Mesh/contact_scaling_bug/01.obj";
        bool texturedModel;
        A->readOBJModel(strA.c_str(), texturedModel, false);

        string strB = "../data/Mesh/contact_scaling_bug/02.obj";
        B->readOBJModel(strB.c_str(), texturedModel, false);

        vector<pPolyMesh> meshes;
        vector<bool> atBoundary;
        meshes.push_back(A);
        meshes.push_back(B);

        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->constructFromPolyMeshes(meshes, atBoundary);

        pPolyMesh contactMesh;
        graph->getContactMesh(contactMesh);

        A->writeOBJModel("01.obj");
        B->writeOBJModel("02.obj");
        contactMesh->writeOBJModel("debug.obj");
    }

}