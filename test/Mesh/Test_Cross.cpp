//
// Created by ziqwang on 2020-02-16.
//

#include <catch2/catch.hpp>
#include "Mesh/Cross.h"
#include "IO/InputVar.h"

TEST_CASE("Cross")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    // Define an Hexagon with 6 pt - Create a cross object + tilt it by 30 degrees

    SECTION("Hexagon"){
        Cross<double> cross(varList);

        int N = 6;
        for(int id = 0; id < N; id++){
            Vector3d pt(std::cos(2 * M_PI / N * id), std::sin(2 * M_PI / N * id), 0);
            cross.push_back(pt);
        }

        float tilt_angle = 30.0;

        // initTiltNormal

        cross.initTiltNormals();
        Cross<double> cross_initial_state = cross;

        for(int id = 0; id < N; id++){
            Vector3d mid = (cross.pos(id) + cross.pos(id + 1)) / 2;
            REQUIRE(Approx(mid.cross(cross.ori(id)->normal).norm()).margin(1e-10) == 0.0);
            REQUIRE(Approx(cross.ori(id)->rotation_angle).margin(1e-10) == 0.0);
        }

        // updateTiltNormalsRoot - Check angle is 30 with phase -1

        cross.updateTiltNormalsRoot(-1.0*tilt_angle);
        for(int id = 0; id < N; id++){
            REQUIRE(Approx(cross.ori(id)->rotation_angle).margin(1e-10) == 30.0);
            REQUIRE(cross.ori(id)->tiltSign == (id % 2 == 0?1 :-1));
        }

        // Check that normal rotated (and in that pi/6 rotation is above/below each of its respective point)

        REQUIRE(Approx((cross.ori(0)->normal - Vector3d( 0.75,  sqrt(3.0)/4.0, -0.5)).norm()).margin(1e-10) == 0.0);
        REQUIRE(Approx((cross.ori(1)->normal - Vector3d( 0.0 ,  sqrt(3.0)/2.0,  0.5)).norm()).margin(1e-10) == 0.0);
        REQUIRE(Approx((cross.ori(2)->normal - Vector3d(-0.75,  sqrt(3.0)/4.0, -0.5)).norm()).margin(1e-10) == 0.0);
        REQUIRE(Approx((cross.ori(3)->normal - Vector3d(-0.75, -sqrt(3.0)/4.0,  0.5)).norm()).margin(1e-10) == 0.0);

        // Back to initial state

        cross.initTiltNormals();
        for(int id = 0; id < N; id++){
            Vector3d mid = (cross.pos(id) + cross.pos(id + 1)) / 2;
            REQUIRE(Approx(mid.cross(cross.ori(id)->normal).norm()).margin(1e-10) == 0.0);
            REQUIRE(Approx(cross.ori(id)->rotation_angle).margin(1e-10) == 0.0);
            REQUIRE(Approx((cross.ori(id)->normal - cross_initial_state.ori(id)->normal).norm()).margin(1e-10) == 0.0);
        }
    }

    SECTION("Four Quad"){
        vector<shared_ptr<Cross<double>>> crossLists;

        int dXY[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};   // 0---3---8
        int verIDs[4][4] = {{0, 1, 2, 3},                   // | 0 | 3 |
                            {1, 4, 5, 2},                   // 1---2---7
                            {2, 5, 6, 7},                   // | 1 | 2 |
                            {3, 2, 7, 8}};                  // 4---5---6


        for(int id = 0; id < 4; id++)
        {
            shared_ptr<Cross<double>> cross = make_shared<Cross<double>>(Cross<double>(varList));
            // vertices position vectors
            cross->push_back(Vector3d(0 + dXY[id][0], 0 + dXY[id][1], 0));
            cross->push_back(Vector3d(1 + dXY[id][0], 0 + dXY[id][1], 0));
            cross->push_back(Vector3d(1 + dXY[id][0], 1 + dXY[id][1], 0));
            cross->push_back(Vector3d(0 + dXY[id][0], 1 + dXY[id][1], 0));

            cross->vers[0]->verID = verIDs[id][0];
            cross->vers[1]->verID = verIDs[id][1];
            cross->vers[2]->verID = verIDs[id][2];
            cross->vers[3]->verID = verIDs[id][3];

            crossLists.push_back(cross);
            cross->crossID = id;
            cross->neighbors.resize(4);
        }
        crossLists[0]->neighbors[1] = crossLists[1];
        crossLists[0]->neighbors[2] = crossLists[3];
        crossLists[1]->neighbors[3] = crossLists[0];
        crossLists[1]->neighbors[2] = crossLists[2];
        crossLists[2]->neighbors[0] = crossLists[1];
        crossLists[2]->neighbors[3] = crossLists[3];
        crossLists[3]->neighbors[1] = crossLists[2];
        crossLists[3]->neighbors[0] = crossLists[0];

        SECTION("getEdgeIDSharedWithCross")
        {
            REQUIRE(crossLists[0]->getEdgeIDSharedWithCross(crossLists[2].get()) == NOT_FOUND);    //  top edge    = 3
            REQUIRE(crossLists[3]->getEdgeIDSharedWithCross(crossLists[1].get()) == NOT_FOUND);    //  left edge   = 0
            REQUIRE(crossLists[0]->getEdgeIDSharedWithCross(crossLists[1].get()) == 1);            //  bottom edge = 1
            REQUIRE(crossLists[0]->getEdgeIDSharedWithCross(crossLists[3].get()) == 2);            //  right edge  = 2
            REQUIRE(crossLists[1]->getEdgeIDSharedWithCross(crossLists[0].get()) == 3);
            REQUIRE(crossLists[1]->getEdgeIDSharedWithCross(crossLists[2].get()) == 2);
            REQUIRE(crossLists[2]->getEdgeIDSharedWithCross(crossLists[1].get()) == 0);
            REQUIRE(crossLists[2]->getEdgeIDSharedWithCross(crossLists[3].get()) == 3);
            REQUIRE(crossLists[3]->getEdgeIDSharedWithCross(crossLists[2].get()) == 1);
            REQUIRE(crossLists[3]->getEdgeIDSharedWithCross(crossLists[0].get()) == 0);
        }

        SECTION("getEdgeIDOfGivenVertexID"){
            REQUIRE(crossLists[1]->getEdgeIDOfGivenVertexID(4) == 1);
            REQUIRE(crossLists[0]->getEdgeIDOfGivenVertexID(3) == 3);
            REQUIRE(crossLists[3]->getEdgeIDOfGivenVertexID(3) == 0);
            REQUIRE(crossLists[0]->getEdgeIDOfGivenVertexID(0) == 0);
            // ensure that NOT_FOUND and -1 are equivalent
            REQUIRE(crossLists[0]->getEdgeIDOfGivenVertexID(6) == NOT_FOUND);
            REQUIRE(crossLists[0]->getEdgeIDOfGivenVertexID(6) == -1);
        }

        SECTION("getPrevEdgeID"){
            REQUIRE(crossLists[1]->getPrevEdgeID(0) == 3);
            REQUIRE(crossLists[0]->getPrevEdgeID(0) == 3);
            REQUIRE(crossLists[0]->getPrevEdgeID(1) == 0);
        }

        SECTION("getCrossIDSharedWithCross"){
            vector<int> shared_crossIDs;
            REQUIRE(crossLists[0]->getCrossIDsSharedWithCross(crossLists[1].get(), shared_crossIDs) == NOT_FOUND);
            REQUIRE(crossLists[0]->getCrossIDsSharedWithCross(crossLists[2].get(), shared_crossIDs) == 2);
            REQUIRE(shared_crossIDs[0] == 1);
            REQUIRE(shared_crossIDs[1] == 3);
            REQUIRE(crossLists[1]->getCrossIDsSharedWithCross(crossLists[3].get(), shared_crossIDs) == 2);
            REQUIRE(shared_crossIDs[0] == 0);
            REQUIRE(shared_crossIDs[1] == 2);
        }

        crossLists[0]->atBoundary = true;
        crossLists[1]->atBoundary = true;
        crossLists[2]->atBoundary = true;
        crossLists[3]->atBoundary = false;
        SECTION("checkNeighborAtBoundary"){
            REQUIRE(crossLists[0]->checkNeighborAtBoundary(0) == true);
            REQUIRE(crossLists[0]->checkNeighborAtBoundary(1) == true);
            REQUIRE(crossLists[0]->checkNeighborAtBoundary(2) == false);
        }
    }
}