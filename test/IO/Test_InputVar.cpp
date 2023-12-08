//
// Created by ziqwang on 2020-02-13.
//

#include "IO/InputVar.h"
#include <catch2/catch_all.hpp>
TEST_CASE("InputVar")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    REQUIRE(varList->varLists.empty());

    SECTION("InputVarLite"){
        InitVar(varList.get());
        REQUIRE(varList->getBool("layerOfBoundary"));
    }

}