//
// Created by ziqwang on 2020-02-22.
//

#include "CrossMesh/PatternCreator.h"
#include <catch2/catch.hpp>
#include <filesystem/path.h>
TEST_CASE("PatternCreator")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    PatternCreator<double> patternCreator(varList);
    PatternCreator<double>::pCrossMesh crossMesh;

    filesystem::create_directory("Pattern");

    SECTION("CROSS_SQUARE"){
        patternCreator.create2DPattern(CROSS_SQUARE, 10, crossMesh);
        crossMesh->writeOBJModel("Pattern/square.obj");
    }
    SECTION("Hexagon"){
        patternCreator.create2DPattern(CROSS_HEXAGON, 10, crossMesh);
        crossMesh->writeOBJModel("Pattern/hexagon.obj");
    }
}