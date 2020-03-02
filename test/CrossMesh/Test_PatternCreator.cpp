//
// Created by ziqwang on 2020-02-22.
//

#include "CrossMesh/PatternCreator.h"
#include <catch2/catch.hpp>
TEST_CASE("PatternCreator")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    PatternCreator<double> patternCreator(varList);
    PatternCreator<double>::pCrossMesh crossMesh;
    //patternCreator.CreateMesh_2DPattern(4, 10, crossMesh);
}