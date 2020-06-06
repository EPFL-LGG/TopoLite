//
// Created by ziqwang on 2020-02-21.
//

#include <catch2/catch.hpp>
#include "Mesh/CrossMesh.h"

TEST_CASE("CrossMesh")
{

    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVar(varList.get());
    PolyMesh<double> polyMesh(varList);

    SECTION("read polyhedron")
    {
        polyMesh.readOBJModel("data/Mesh/primitives/Icosphere.obj", true);
        CrossMesh<double> crossMesh(polyMesh);
        REQUIRE(crossMesh.getVertices().size() == polyMesh.vertexList.size());
        REQUIRE(Approx(crossMesh.volume()) == polyMesh.volume());
        REQUIRE(crossMesh.getVarList()->getFloat("tiltAngle") == Approx(20.0));
    }

    SECTION("test neighbors")
    {
        vector<shared_ptr<_Polygon<double>> > polyLists;

        int XYZ[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                         {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};

        int face[6][4] = {{0, 3, 2, 1},
                          {6, 5, 1, 2},
                          {2, 3, 7, 6},
                          {0, 4, 7, 3},
                          {0, 1, 5, 4},
                          {4, 5, 6, 7}};

        for(int id = 0;id < 6; id++)
        {
            shared_ptr<_Polygon< double>> poly = make_shared<_Polygon < double>> (_Polygon<double>());
            for(int jd = 0; jd < 4; jd++)
            {
                Vector3d pt(XYZ[face[id][jd]][0], XYZ[face[id][jd]][1], XYZ[face[id][jd]][2]);
                poly->push_back(pt);
            }
            polyLists.push_back(poly);
        }

        polyMesh.setPolyLists(polyLists);
        CrossMesh<double> crossMesh(polyMesh);
        REQUIRE(crossMesh.getVertices().size() == 8);

        // neighbor
        REQUIRE(crossMesh.cross(0)->neighbors[0].lock()->crossID == 3);
        REQUIRE(crossMesh.cross(0)->neighbors[3].lock()->crossID == 4);
    }
}