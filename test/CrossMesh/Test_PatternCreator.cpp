//
// Created by ziqwang on 2020-02-22.
//

#include "CrossMesh/PatternCreator.h"
#include <catch2/catch.hpp>

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesysten>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

TEST_CASE("PatternCreator")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());
    PatternCreator<double> patternCreator(varList);
    PatternCreator<double>::pCrossMesh crossMesh;

    std::filesystem::create_directory("Pattern");

    SECTION("CROSS_SQUARE"){
        patternCreator.create2DPattern(CROSS_SQUARE, 10, crossMesh);
        crossMesh->writeOBJModel("Pattern/square.obj");
    }
    SECTION("CROSS_HEXAGON"){
        patternCreator.create2DPattern(CROSS_HEXAGON, 10, crossMesh);
        crossMesh->writeOBJModel("Pattern/hexagon.obj");
    }
    SECTION("CROSS_RHOMBUS"){
        patternCreator.create2DPattern(CROSS_RHOMBUS, 10, crossMesh);
        crossMesh->writeOBJModel("Pattern/rhombus.obj");
    }
}