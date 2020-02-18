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
    Cross<double> cross(varList);

    cross.print();

    SECTION("Hexagon"){
        double radius = 1.0;
        int N = 6;
        for(int id = 0; id < N; id++){
            Vector3d pt(std::cos(2 * M_PI / N * id), std::sin(2 * M_PI / N * id), 0);
            cross.push_back(pt);
        }

        cross.initTiltNormals();
        for(int id = 0; id < N; id++){
            Vector3d mid = (cross[id] + cross[id + 1]) / 2;
            REQUIRE(Approx(mid.cross(cross.ori(id)->normal).norm()).margin(1e-10) == 0.0);
        }
        
    }
}