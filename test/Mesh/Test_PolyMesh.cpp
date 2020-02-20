//
// Created by ziqwang on 2020-02-20.
//

#include <catch2/catch.hpp>
#include "Mesh/PolyMesh.h"

TEST_CASE("PolyMesh")
{
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    PolyMesh<double> polyMesh(varList);

    SECTION("Four Quad with index as input") {
        vector<shared_ptr<_Polygon<double>> > polyLists;

        int dXY[4][2] = {{0, 0},
                         {1, 0},
                         {1, 1},
                         {0, 1}};
        int verIDs[4][4] = {{0, 1, 2, 3},
                            {1, 4, 5, 2},
                            {2, 5, 6, 7},
                            {3, 2, 7, 8}};

        for (int id = 0; id < 4; id++)
        {
            shared_ptr<_Polygon< double>> poly = make_shared<_Polygon < double>> (_Polygon<double>());
            poly->push_back(Vector3d(0 + dXY[id][0], 0 + dXY[id][1], 0));
            poly->push_back(Vector3d(1 + dXY[id][0], 0 + dXY[id][1], 0));
            poly->push_back(Vector3d(1 + dXY[id][0], 1 + dXY[id][1], 0));
            poly->push_back(Vector3d(0 + dXY[id][0], 1 + dXY[id][1], 0));

            poly->vers[0]->verID = verIDs[id][0];
            poly->vers[1]->verID = verIDs[id][1];
            poly->vers[2]->verID = verIDs[id][2];
            poly->vers[3]->verID = verIDs[id][3];

            polyLists.push_back(poly);
        }

        polyMesh.setPolyLists(polyLists);

        REQUIRE((polyMesh.vertexList[4]->pos - Vector3d(2, 0, 0)).norm() == Approx(0.0));
        REQUIRE((polyMesh.vertexList[5]->pos - Vector3d(2, 1, 0)).norm() == Approx(0.0));

        REQUIRE((polyMesh.bbox.minPt - Vector3d(0, 0, 0)).norm() == Approx(0.0));
        REQUIRE((polyMesh.bbox.maxPt - Vector3d(2, 2, 0)).norm() == Approx(0.0));
        REQUIRE((polyMesh.bbox.cenPt - Vector3d(1, 1, 0)).norm() == Approx(0.0));
        REQUIRE((polyMesh.bbox.size - Vector3d(2, 2, 0)).norm() == Approx(0.0));

        REQUIRE((polyMesh.centroid - Vector3d(0, 0, 0)).norm() == Approx(0.0));

        REQUIRE(polyMesh.volume == Approx(0.0));

        REQUIRE((polyMesh.lowestPt - Vector3d(0, 0, 0)).norm() == Approx(0.0));
    }
}