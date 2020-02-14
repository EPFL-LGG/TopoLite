//
// Created by ziqwang on 14.12.19.
//
#include "Mesh/PolyMesh.h"
#include <catch2/catch.hpp>
using pPolyMesh = shared_ptr<PolyMesh>;

TEST_CASE("Class PolyMesh")
{

    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    SECTION("Update Vertices")
    {
        pPolygon PA = make_shared<_Polygon>();
        PA->push_back(Vector3f(0, 0, 0));
        PA->push_back(Vector3f(2, 0, 0));
        PA->push_back(Vector3f(2, 2, 0));
        PA->push_back(Vector3f(0, 2, 0));

        pPolygon PB = make_shared<_Polygon>();
        PB->push_back(Vector3f(0, 0, 0));
        PB->push_back(Vector3f(1, 0, 0));
        PB->push_back(Vector3f(1, 1, 0));
        PB->push_back(Vector3f(0, 1, 0));

        pPolyMesh polyMesh = make_shared<PolyMesh>(varList);
        polyMesh->polyList.push_back(PA);
        polyMesh->polyList.push_back(PB);

        polyMesh->removeDuplicatedVertices();

        REQUIRE(polyMesh->vertexList.size() == 7);
        REQUIRE(polyMesh->polyList[0]->verIDs[0] == 0);
        REQUIRE(polyMesh->polyList[0]->verIDs[1] == 1);
        REQUIRE(polyMesh->polyList[1]->verIDs[0] == 0);
        REQUIRE(polyMesh->polyList[1]->verIDs[3] == 6);
    }
}