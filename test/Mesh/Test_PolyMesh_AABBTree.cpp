//
// Created by ziqwang on 2020-02-22.
//


#include <catch2/catch.hpp>
#include <TopoLite/Mesh/PolyMesh_AABBTree.h>
using Eigen::Vector3d;
using Eigen::Vector2d;
TEST_CASE("PolyMesh_AABBTree")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    SECTION("read obj"){
        bool texturedModel;
        polyMesh->readOBJModel("../data/Mesh/primitives/Icosphere.obj", texturedModel, true);
        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        aabbTree.findTexPoint(Vector2d(0.2, 0.2));
    }

    SECTION("one triangle"){
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0.5, 0.5));
        vector<shared_ptr<_Polygon<double>>> polyLists;
        polyLists.push_back(poly);
        polyMesh->setPolyLists(polyLists);

        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.2, 0.2)) != nullptr);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0)) == polyMesh->polyList[0]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.6, 0.5)) == nullptr);
    }

    SECTION("two triangles"){
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0.5, 0.5));
        vector<shared_ptr<_Polygon<double>>> polyLists;
        polyLists.push_back(poly);

        poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0.5));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0, 0.5));
        polyLists.push_back(poly);

        polyMesh->setPolyLists(polyLists);

        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0.3)) == polyMesh->polyList[1]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.1, 0)) == polyMesh->polyList[0]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0)) != nullptr);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.5, 0.5)) != nullptr);
    }

}