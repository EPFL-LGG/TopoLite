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

    SECTION("Hexagon"){
        Cross<double> cross(varList);


        double radius = 1.0;
        int N = 6;
        for(int id = 0; id < N; id++){
            Vector3d pt(std::cos(2 * M_PI / N * id), std::sin(2 * M_PI / N * id), 0);
            cross.push_back(pt);
        }


        //initTiltNormal
        cross.initTiltNormals();
        for(int id = 0; id < N; id++){
            Vector3d mid = (cross[id] + cross[id + 1]) / 2;
            REQUIRE(Approx(mid.cross(cross.ori(id)->normal).norm()).margin(1e-10) == 0.0);
        }

        //updateTiltNormalsRoot
        cross.updateTiltNormalsRoot(-30);
        for(int id = 0; id < N; id++){
            REQUIRE(Approx(cross.ori(id)->rotation_angle).margin(1e-10) == 30.0);
            REQUIRE(cross.ori(id)->tiltSign == (id % 2 == 0?1 :-1));
        }
        REQUIRE(Approx((cross.ori(0)->normal - Vector3d(3.0/4, sqrt(3)/4, -1.0 / 2)).norm()).margin(1e-10) == 0.0);

        //updateTiltNormals
        cross.updateTiltNormals(30);
    }

    SECTION("Four Quad"){
        vector<Cross<double>> crossLists;

        int dXY[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        for(int id = 0; id < 4; id++)
        {
            Cross<double> cross(varList);
            cross.push_back(Vector3d(0 + dXY[id][0], 0 + dXY[id][1], 0));
            cross.push_back(Vector3d(1 + dXY[id][0], 0 + dXY[id][1], 0));
            cross.push_back(Vector3d(1 + dXY[id][0], 1 + dXY[id][1], 0));
            cross.push_back(Vector3d(0 + dXY[id][0], 1 + dXY[id][1], 0));
            crossLists.push_back(cross);
        }
    }
}